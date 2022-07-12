//
// Created by Duan Bin on 2022/1/8.
//

#ifndef MEASUREMENT_PACKAGE_H
#define MEASUREMENT_PACKAGE_H

#include <vector>
#include "Eigen/Dense"

class MeasurementPackage {
public:
    long long timestamp_;

    enum SensorType{
        LASER,
        RADAR
    } sensor_type_;

    Eigen::VectorXd raw_measurements_;

    std::vector<MeasurementPackage> meas_pack_list; // List of measurement package
};

#endif //MEASUREMENT_PACKAGE_H
