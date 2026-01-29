#include "z_se_planning/bspline_optimizer.h"

void BsplineOptimizer::calcDynamicCost(const std::vector<Eigen::Vector3d>& q, 
                                       double& cost, 
                                       std::vector<Eigen::Vector3d>& gradient) {
    cost = 0.0;
    // Iterate over all control points of the B-Spline
    for (int i = 0; i < q.size(); ++i) {
        double t_curr = i * bspline_interval_; // Time at this control point
        
        // --- Latent State Extrapolation ---
        // Predict dynamic object position at time t_curr based on Memory
        // predicted_obj_traj_ is updated by the perception module (Kalman/Velocity model)
        Eigen::Vector3d obj_pos_t = predictObstaclePos(t_curr);

        Eigen::Vector3d dist_vec = q[i] - obj_pos_t;
        double dist_sq = dist_vec.squaredNorm();
        double safe_sq = safe_radius_ * safe_radius_;

        // --- Semantic Safety Constraint (Soft Penalty) ---
        // Eq. 6: Psi(R^2 - d^2)
        if (dist_sq < safe_sq) {
            // Penalty function: Cubic for smoothness
            double pen = pow(safe_sq - dist_sq, 3); 
            cost += weight_dynamic_ * pen;

            // Compute Gradient for Optimization (L-BFGS)
            // d(Cost)/d(q) = d(Psi)/d(d^2) * d(d^2)/d(q)
            //              = -3 * (R^2 - d^2)^2 * 2 * (q - obj)
            double grad_coeff = -3.0 * pow(safe_sq - dist_sq, 2);
            gradient[i] += weight_dynamic_ * grad_coeff * 2.0 * dist_vec;
        }
    }
}