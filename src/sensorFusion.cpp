//
// Created by Duan Bin on 2022/1/8.
//

#include <iostream>
#include "sensorFusion.h"
using namespace std;

SensorFusion::SensorFusion() {
    is_initialized_ = false;
    last_timestamp_ = 0.0;

    // 初始化激光雷达的测量矩阵 H_lidar_
    H_lidar_ = Eigen::MatrixXd(2, 4);
    H_lidar_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    // 设置传感器的测量噪声矩阵
    R_lidar_ = Eigen::MatrixXd(2, 2);
    R_lidar_ << 0.0225, 0,
                0, 0.0225;

    R_radar_ = Eigen::MatrixXd(3, 3);
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;
}

SensorFusion::~SensorFusion() {

}

void SensorFusion::Process(MeasurementPackage measurement_pack) {

    if (!is_initialized_){
        Eigen::Vector4d x;
        if (measurement_pack.sensor_type_ == MeasurementPackage::LASER){
            // 如果第一帧数据是激光雷达数据，没有速度信息，因此初始化时只能传入位置，速度设置为0
            x << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
            cout<<x(0)<<' '<<x(1)<<endl;
        } else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR){
            // 如果第一帧数据是毫米波雷达，可以通过三角函数算出x-y坐标系下的位置和速度
            double rho = measurement_pack.raw_measurements_[0];
            double phi = measurement_pack.raw_measurements_[1];
            double rho_dot = measurement_pack.raw_measurements_[2];
            double position_x = rho * cos(phi);
            if (position_x < 0.0001){
                position_x = 0.0001;
            }
            double position_y = rho * sin(phi);
            if (position_y < 0.0001){
                position_y = 0.0001;
            }
            double velocity_x = rho_dot * cos(phi);
            double velocity_y = rho_dot * sin(phi);
            x << position_x, position_y, velocity_x, velocity_y;
        }

        // 避免运算时，0作为被除数
        if (fabs(x(0)) < 0.001){
            x(0) = 0.001;
        }
        if (fabs(x(1)) < 0.001){
            x(1) = 0.001;
        }
        // 初始化Kalman滤波器
        kf_.Initialization(x);

        if (!kf_.IsInitialized()){
            std::cout<<"uninit"<<std::endl;
        } else{
            std::cout<<"init"<<' '<<x(0)<<' '<<x(1)<<std::endl;
        }

        // 设置协方差矩阵P
        Eigen::MatrixXd P = Eigen::MatrixXd(4, 4);
        P << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1000.0, 0.0,
             0.0, 0.0, 0.0, 1000.0;
        kf_.SetP(P);

        // 设置过程噪声Q
        Eigen::MatrixXd Q = Eigen::MatrixXd(4, 4);
        Q << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0;
        kf_.SetQ(Q);

        // 存储第一帧的时间戳，供下一帧数据使用
        last_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;
        return;
    }

    double delta_t = (measurement_pack.timestamp_ - last_timestamp_) / 1000000.0;
//    cout<<delta_t<<endl;
    last_timestamp_ = measurement_pack.timestamp_;

//    double delta_t = 0.99;
//    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR){
//        delta_t = 0.01;
//    }

    Eigen::MatrixXd F = Eigen::MatrixXd(4, 4);
    F << 1.0, 0.0, delta_t, 0.0,
         0.0, 1.0, 0.0, delta_t,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0;
    kf_.SetF(F);

    kf_.Prediction();
//    cout<<kf_.GetX()<<endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::LASER){
        kf_.SetH(H_lidar_);
        kf_.SetR(R_lidar_);
        kf_.KFUpdate(measurement_pack.raw_measurements_);
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR){
        kf_.SetR(R_radar_);
        kf_.EKFUpdate(measurement_pack.raw_measurements_);
    }
}
