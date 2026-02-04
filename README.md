## S-FUEL Exploration

### First-person and Third-person perspective(Find Orange) Demo
<img src="results_visual/gifs/First-person and Third-person perspective (Find Orange).gif" width="800"/>


### FUEL(Baseline) with Third-person perspective(No Goal) Demo
<img src="results_visual/gifs/FUEL(Baseline).gif" width="800"/>


### First-person perspective(Find Bottle) Demo
<img src="results_visual/gifs/First-person perspective(Find Bottle).gif" width="800"/>


### First-person perspective(Find Chair) Demo
<img src="results_visual/gifs/First-person perspective(Find Chair).gif" width="800"/>


### First-person perspective(Find Orange) Demo
<img src="results_visual/gifs/First-person perspective(Find Orange).gif" width="800"/>


#### high_fast_lio2
High-frequency radar odometer, Features include: 1. Odometry frequency is comparable to radar IMU frequency (200Hz) through IMU interpolation; 2. Mapping the odometry point to the vision pose enables fixed-point flight in Position mode.
The use of a 200 Hz high-frequency radar odometry is specifically designed to match the px4ctrl control framework, which requires high-rate state estimation feedback for stable and responsive control. Providing odometry at the same frequency as the IMU significantly reduces control latency, improves state consistency, and ensures smooth integration with the px4ctrl position controller.

#### realsense-ros
ROS driver module for RealSense D435i camera

#### px4ctrl
The controller px4ctrl is designed based on the DJIN3 flight controller N3ctrl, with modifications to the controller and state machine. For reference, see the N3Ctrl code.

#### src/FUEL
Baseline implementation adapted from FUEL.

#### src/mapping
Global map creation and understanding

#### src/perception
Combining SAM3's semantic awareness

#### src/planner
Zero-shot target programming combined with perceptual understanding


#######################################################################
## livox driver
source FAST_LIO2/fastlio2/devel/setup.bash 
roslaunch livox_ros_driver2 msg_MID360.launch 

## fastlio2  and odometry remap to vision_pose
source FAST_LIO2/fastlio2/devel/setup.bash 
roslaunch fast_lio mapping_mid360.launch

## px4  mavros for imu 
roslaunch mavros px4.launch

# realsense d435i camera 
source realsense_ws/devel/setup.sh  
roslaunch realsense2_camera rs_camera.launch

## px4ctrl  node
roslaunch px4ctrl run_ctrl.launch


## fuel 
source catkin_fuel_ws/devel/setup.bash
roslaunch exploration_manager exploration.launch

## ego_swarm 
source catkin_ego_swarm_ws/devel/setup.sh 
roslaunch ego_planner run_in_exp.launch
# if use lidar build the map (choose)， if use camera ,not run this line （from camera_init to world）
roslaunch ego_planner mid360_to_octomap.launch 

## take off 
cd px4ctrl_ws/shfiles/

sh takeoff.sh

sh land.sh

## rc mode 
1: stabale
2: position
3: offboard













