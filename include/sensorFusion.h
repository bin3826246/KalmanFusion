//
// Created by Duan Bin on 2022/1/8.
//

#ifndef SENSORFUSION_H
#define SENSORFUSION_H

#include "measurement_package.h"
#include "kalmanfilter.h"

class SensorFusion {
public:
    SensorFusion();
    ~SensorFusion();

    void Process(MeasurementPackage measurement_pack);
    KalmanFilter kf_;

private:
    bool is_initialized_;
    long long last_timestamp_;
    Eigen::MatrixXd R_lidar_;
    Eigen::MatrixXd R_radar_;
    Eigen::MatrixXd H_lidar_;
};


#endif //SENSORFUSION_H
