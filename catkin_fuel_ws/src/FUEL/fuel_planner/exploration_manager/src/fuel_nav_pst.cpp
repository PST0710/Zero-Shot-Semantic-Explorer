/*****************************************************************************************
 * 自定义控制器跟踪egoplanner轨迹
 * 本代码采用的mavros的速度控制进行跟踪
 * 编译成功后直接运行就行，遥控器先position模式起飞，然后rviz打点，再切offborad模式即可
 ******************************************************************************************/
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/RCIn.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <limits>
#include <cstdint>
// PositionTarget type_mask bits (from LSB=bit0):
// bit0 PX, bit1 PY, bit2 PZ, bit3 VX, bit4 VY, bit5 VZ,
// bit6 AX, bit7 AY, bit8 AZ, bit9 FORCE, bit10 YAW, bit11 YAW_RATE
// We want to USE VX,VY,VZ and YAW => set ignore bits for all others (1 = ignore)
static constexpr uint16_t VELOCITY2D_CONTROL = 
    (1u<<0)  /*IGNORE_PX*/ |
    (1u<<1)  /*IGNORE_PY*/ |
    (1u<<2)  /*IGNORE_PZ*/ |
    (1u<<6)  /*IGNORE_AFX*/|
    (1u<<7)  /*IGNORE_AFY*/|
    (1u<<8)  /*IGNORE_AFZ*/|
    (1u<<9)  /*FORCE*/    |
    (1u<<11) /*IGNORE_YAW_RATE*/;
class Ctrl
{
    public:
        Ctrl();
        void state_cb(const mavros_msgs::State::ConstPtr &msg);
        void position_cb(const nav_msgs::Odometry::ConstPtr &msg);
        void target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
        void control(const ros::TimerEvent&);
        ros::NodeHandle nh;
        // configurable params
        std::string odom_topic, ego_topic, setpoint_topic, marker_topic;
        bool odom_frame_is_enu; // true if odom from fast-lio2 is ENU
        double pos_kp; // P gain for position->velocity
        double vel_blend; // blending factor for ego velocity
        double max_vel; // max velocity magnitude
        visualization_msgs::Marker trackpoint;
        quadrotor_msgs::PositionCommand ego;
        tf::StampedTransform ts;//用来发布无人机当前位置的坐标系坐标轴
        tf::TransformBroadcaster tfBroadcasterPointer;	//广播坐标轴
        unsigned short velocity_mask = VELOCITY2D_CONTROL;
        mavros_msgs::PositionTarget current_goal;
        mavros_msgs::RCIn rc;
        nav_msgs::Odometry position_msg;
        geometry_msgs::PoseStamped target_pos;
        mavros_msgs::State current_state;
        float position_x, position_y, position_z, now_x, now_y, now_yaw, current_yaw, targetpos_x, targetpos_y;
        float ego_pos_x, ego_pos_y, ego_pos_z, ego_vel_x, ego_vel_y, ego_vel_z, ego_a_x, ego_a_y, ego_a_z, ego_yaw, ego_yaw_rate; //EGO planner information has position velocity acceleration yaw yaw_dot
        bool receive, get_now_pos;//触发轨迹的条件判断
        ros::Subscriber state_sub, twist_sub, target_sub, position_sub;
        ros::Publisher local_pos_pub, pubMarker;
        ros::Timer timer;
};
Ctrl::Ctrl()
{
    ros::NodeHandle pnh("~");
    // load parameters with sensible defaults
    pnh.param<std::string>("odom_topic", odom_topic, std::string("/Odometry"));
    pnh.param<std::string>("ego_topic", ego_topic, std::string("/planning/pos_cmd"));
    pnh.param<std::string>("setpoint_topic", setpoint_topic, std::string("/uav1/mavros/setpoint_raw/local"));
    pnh.param<std::string>("marker_topic", marker_topic, std::string("/track_drone_point"));
    pnh.param<bool>("odom_frame_is_enu", odom_frame_is_enu, true);
    // default gains are conservative; can be overridden via rosparam
    pnh.param<double>("pos_kp", pos_kp, 0.6);
    pnh.param<double>("vel_blend", vel_blend, 0.5);
    pnh.param<double>("max_vel", max_vel, 1.2);

    ROS_INFO("fuel_nav: odom_topic=%s, ego_topic=%s, setpoint_topic=%s, odom_frame_is_enu=%s", odom_topic.c_str(), ego_topic.c_str(), setpoint_topic.c_str(), odom_frame_is_enu?"true":"false");

    timer = nh.createTimer(ros::Duration(0.02), &Ctrl::control, this);
    state_sub = nh.subscribe("/uav1/mavros/state", 10, &Ctrl::state_cb, this);
    position_sub=nh.subscribe(odom_topic, 10, &Ctrl::position_cb, this);
    target_sub = nh.subscribe("move_base_simple/goal", 10, &Ctrl::target_cb, this);
    twist_sub = nh.subscribe(ego_topic, 10, &Ctrl::twist_cb, this);
    local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>(setpoint_topic, 1);
    pubMarker = nh.advertise<visualization_msgs::Marker>(marker_topic, 5);
    get_now_pos = false;
    receive = false;
}
void Ctrl::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}

//read vehicle odometry
void Ctrl::position_cb(const nav_msgs::Odometry::ConstPtr&msg)
{
    position_msg=*msg;
	tf2::Quaternion quat;
	tf2::convert(msg->pose.pose.orientation, quat); //把mavros/local_position/pose里的四元数转给tf2::Quaternion quat
	double roll, pitch, yaw;
	tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    ts.stamp_ = msg->header.stamp;
    // use odom header frame to keep TF consistent if available
    ts.frame_id_ = msg->header.frame_id.size()?msg->header.frame_id:std::string("world");
    ts.child_frame_id_ = std::string("drone_frame");
    ts.setRotation(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
    ts.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    tfBroadcasterPointer.sendTransform(ts);
	if (!get_now_pos) 
	{
		now_x = position_msg.pose.pose.position.x;
		now_y = position_msg.pose.pose.position.y;
		tf2::Quaternion quat;
		tf2::convert(msg->pose.pose.orientation, quat);
		now_yaw = yaw;
		get_now_pos = true;
	}
    position_x = position_msg.pose.pose.position.x;
    position_y = position_msg.pose.pose.position.y;
    position_z = position_msg.pose.pose.position.z;
	current_yaw = yaw;

}

void Ctrl::target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)//读取rviz的航点
{
    receive = true;
    target_pos = *msg;
    targetpos_x = target_pos.pose.position.x;
    targetpos_y = target_pos.pose.position.y;
}

//读取ego里的位置速度加速度yaw和yaw-dot信息，其实只需要ego的位置速度和yaw就可以了
void Ctrl::twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)//ego的回调函数
{
	ego = *msg;
    ego_pos_x = ego.position.x;
	ego_pos_y = ego.position.y;
	ego_pos_z = ego.position.z;
	ego_vel_x = ego.velocity.x;
	ego_vel_y = ego.velocity.y;
	ego_vel_z = ego.velocity.z;
	ego_yaw = ego.yaw;
	ego_yaw_rate = ego.yaw_dot;
}

void Ctrl::control(const ros::TimerEvent&)
{
    // safety: require mavros connection
    if(!current_state.connected)
    {
        ROS_WARN_THROTTLE(5.0, "mavros not connected yet - not publishing setpoints");
        return;
    }
    if(!receive) //如果没有在rviz上打点，则offboard模式下会保持在1m的高度
    {
        // reset/clear previous fields to avoid stale values
        current_goal = mavros_msgs::PositionTarget();
        current_goal.header.stamp = ros::Time::now();
        current_goal.header.frame_id = position_msg.header.frame_id.size()?position_msg.header.frame_id:std::string("local_origin");
        current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_goal.type_mask = velocity_mask;
        // P controller for position -> velocity, using parameter pos_kp
        double vx = pos_kp * (now_x - position_x);
        double vy = pos_kp * (now_y - position_y);
        double vz = pos_kp * (1.0 - position_z);
        // check finite
        if(!std::isfinite(vx)) vx = 0.0;
        if(!std::isfinite(vy)) vy = 0.0;
        if(!std::isfinite(vz)) vz = 0.0;
        // MAVROS自动将ENU转换为NED,无需手动转换
        // clamp
        double vm = sqrt(vx*vx + vy*vy + vz*vz);
        if(vm > max_vel && vm > 1e-6)
        {
            double s = max_vel / vm;
            vx *= s; vy *= s; vz *= s;
        }
        current_goal.velocity.x = vx;
        current_goal.velocity.y = vy;
        current_goal.velocity.z = vz;
        current_goal.yaw = now_yaw;
        ROS_INFO("请等待");
    }

    //if receive plan in rviz, the EGO plan information can input mavros and vehicle can auto navigation
    if(receive)//触发后进行轨迹跟踪
    {
        // reset/clear previous fields to avoid stale values
        current_goal = mavros_msgs::PositionTarget();
        current_goal.header.stamp = ros::Time::now();
        current_goal.header.frame_id = position_msg.header.frame_id.size()?position_msg.header.frame_id:std::string("local_origin");
        current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;//选择local系，一定要local系
        current_goal.type_mask = velocity_mask;//这个就算对应的掩码设置，可以看mavros_msgs::PositionTarget消息格式
        // blend ego velocity and P-term on position error
        double vx = vel_blend * ego_vel_x + pos_kp * (ego_pos_x - position_x);
        double vy = vel_blend * ego_vel_y + pos_kp * (ego_pos_y - position_y);
        double vz = pos_kp * (ego_pos_z - position_z);
        // NaN guard
        if(!std::isfinite(vx)) vx = 0.0;
        if(!std::isfinite(vy)) vy = 0.0;
        if(!std::isfinite(vz)) vz = 0.0;
        // MAVROS自动将ENU转换为NED,无需手动转换
        // clamp overall velocity
        double vm = sqrt(vx*vx + vy*vy + vz*vz);
        if(vm > max_vel && vm > 1e-6)
        {
            double s = max_vel / vm;
            vx *= s; vy *= s; vz *= s;
        }
        current_goal.velocity.x = vx;
        current_goal.velocity.y = vy;
        current_goal.velocity.z = vz;
        current_goal.yaw = ego_yaw;
        ROS_INFO("EGO规划速度：vel = %.2f", sqrt(vx*vx + vy*vy));
    }
    local_pos_pub.publish(current_goal);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cxr_egoctrl_v1");
	setlocale(LC_ALL,"");
    Ctrl ctrl;
    ros::spin();
	return 0;
}
