//
// Created by Duan Bin on 2022/1/7.
//
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include "ground_truth_package.h"
#include "measurement_package.h"
#include "sensorFusion.h"

int main(){

    std::string input_file_name = "../data/data-1.txt"; //sample-laser-radar-measurement-data-2.txt

    std::ifstream input_file(input_file_name.c_str(), std::ifstream::in);
    if (!input_file.is_open()){
        std::cout << "failed to open data file" << std::endl;
        return -1;
    }

    // measurement_pack_list：毫米波雷达/激光雷达实际测得的数据。数据包含测量值和时间戳，即融合算法的输入。
    // groundtruth_pack_list：每次测量时，障碍物位置的真值。对比融合算法输出和真值的差别，用于评估融合算法结果的好坏。
    std::vector<MeasurementPackage> measurement_pack_list;
    std::vector<GroundTruthPackage> groundtruth_pack_list;

    // 通过while循环将雷达测量值和真值全部读入内存，存入measurement_pack_list和groundtruth_pack_list中
    std::string line;
    while (std::getline(input_file, line)){
        std::string sensor_type;
        MeasurementPackage meas_package;
        GroundTruthPackage gt_package;
        std::istringstream iss(line);
        long long timestamp;

        // 读取当前行的第一个元素，L代表Lidar数据，R代表Radar数据
        iss >> sensor_type;
        if (sensor_type == "L"){
//            continue;
            // 激光雷达数据 Lidar data
            // 该行第二个元素为测量值x，第三个元素为测量值y，第四个元素为时间戳(纳秒）
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = Eigen::VectorXd(2);
            double x;
            double y;
            iss >> x;
            iss >> y;
            meas_package.raw_measurements_ << x, y;
            iss >> timestamp;
//            std::cout<<meas_package.raw_measurements_[0]<<' '<<x<<std::endl;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        } else if (sensor_type == "R"){
//            continue;
            // 毫米波雷达数据 Radar data
            // 该行第二个元素为距离pho，第三个元素为角度phi，第四个元素为径向速度pho_dot，第五个元素为时间戳(纳秒）
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = Eigen::VectorXd(3);
            double rho;
            double phi;
            double rho_dot;
            iss >> rho;
            iss >> phi;
            iss >> rho_dot;
            meas_package.raw_measurements_ << rho, phi, rho_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        }

        // 当前行的最后四个元素分别是x方向上的距离真值，y方向上的距离真值，x方向上的速度真值，y方向上的速度真值
        double x_gt;
        double y_gt;
        double vx_gt;
        double vy_gt;
        iss >> x_gt;
        iss >> y_gt;
        iss >> vx_gt;
        iss >> vy_gt;
        gt_package.gt_values_ = Eigen::VectorXd(4);
        gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
        groundtruth_pack_list.push_back(gt_package);
    }

    std::cout << "Success to load data." << std::endl;

    // 部署跟踪算法
    SensorFusion fuser;

    int count = 0;
    for (int i = 0; i < measurement_pack_list.size(); i++) {
//        if (count > 3) break;
        count++;
        fuser.Process(measurement_pack_list[i]);
        Eigen::Vector4d x_out = fuser.kf_.GetX();

        std::cout << count << " x " << x_out(0) << ' ' << x_out(0) - groundtruth_pack_list[i].gt_values_(0)
                  << " y " << x_out(1) << ' ' << x_out(1) - groundtruth_pack_list[i].gt_values_(1)
                  << " vx " << x_out(2) << ' ' << x_out(2) - groundtruth_pack_list[i].gt_values_(2)
                  << " vy " << x_out(3) << ' ' << x_out(3) - groundtruth_pack_list[i].gt_values_(3)
                  << std::endl;

    }

}
