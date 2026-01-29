#include "z_se_planning/frontier_finder.h"

// Core Utility Function: Combines Geometric Entropy and Semantic KL Divergence
double FrontierFinder::computeFrontierInformation(const Eigen::Vector3d& center, 
                                                  double radius, 
                                                  const std::string& target_class) {
    // 1. Geometric Information Gain (Shannon Entropy approximation)
    // Corresponds to the volume of unmapped space visible from 'center'
    double geom_gain = 0.0;
    std::vector<Eigen::Vector3d> visible_voxels = raycastFrustum(center);
    
    for (const auto& pt : visible_voxels) {
        if (map_->getOccupancy(pt) == MapState::UNKNOWN) {
            geom_gain += 1.0; 
        }
    }

    // 2. Semantic Information Gain (KL Divergence Reduction)
    // We want to go to places where the semantic probability is HIGH for the target.
    // Minimizing KL(Q || P) is equivalent to Maximizing Cross-Entropy.
    // Here we implement the weighted utility term directly.
    double sem_gain = 0.0;
    
    for (const auto& pt : visible_voxels) {
        // Get P(c | map)
        double prob = semantic_map_->getProbability(pt, target_class);
        
        // If the map hints that this area might contain the target (prob > 0.5),
        // we increase the gain to encourage investigation.
        if (prob > 0.5) {
            // Log-likelihood ratio acts as the utility
            sem_gain += prob * std::log(prob / (1.0 - prob));
        }
    }

    // 3. Final Utility Calculation (Eq. 4 in Paper)
    // lambda_geo_ and lambda_sem_ are hyperparameters (e.g., 1.0 and 2.5)
    double total_utility = lambda_geo_ * geom_gain + lambda_sem_ * sem_gain;

    // Optional: Add Navigation Cost Penalty (- beta * Cost)
    double nav_cost = (center - current_drone_pos_).norm();
    total_utility -= beta_cost_ * nav_cost;

    return total_utility;
}