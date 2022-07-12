//
// Created by Duan Bin on 2022/1/8.
//

#ifndef GROUND_TRUTH_PACKAGE_H
#define GROUND_TRUTH_PACKAGE_H

#include <vector>
#include "Eigen/Dense"

class GroundTruthPackage {
public:
    long timestamp_;

    enum SensorType{
        LASER,
        RADAR
    } sensor_type_;

    Eigen::VectorXd gt_values_;

    std::vector<GroundTruthPackage> gt_pack_list; // List of gt package

};

#endif //GROUND_TRUTH_PACKAGE_H
