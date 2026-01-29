#include "z_se_mapping/semantic_map.h"

// Eq. 2: Recursive Log-Odds Update
// L_t = L_{t-1} + log(p/(1-p)) - L_0
void SemanticMap::updateVoxelBelief(const Eigen::Vector3d& point, 
                                    const std::string& class_label, 
                                    double sensor_confidence) {
    Voxel* voxel = grid_->getVoxel(point);
    if (!voxel) return;

    // 1. Compute Inverse Sensor Model Term
    // Heuristic: Confidence scales with sensor score and decays with distance
    double p_z = std::min(0.9, std::max(0.1, sensor_confidence)); 
    double log_odds_meas = log(p_z / (1.0 - p_z));

    // 2. Retrieve Current Log-Odds
    double L_prev = voxel->getLogOdds(class_label);

    // 3. Apply Update
    double L_new = L_prev + log_odds_meas - prior_log_odds_;
    
    // Clamp to prevent numerical instability
    L_new = std::max(-10.0, std::min(10.0, L_new));

    voxel->setLogOdds(class_label, L_new);
}

double SemanticMap::getProbability(const Eigen::Vector3d& point, 
                                   const std::string& target_class) {
    Voxel* voxel = grid_->getVoxel(point);
    if (!voxel) return 0.5; // Unknown

    double log_odds = voxel->getLogOdds(target_class);
    return 1.0 / (1.0 + exp(-log_odds)); // Sigmoid conversion
}