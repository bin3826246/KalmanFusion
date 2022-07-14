//
// Created by Duan Bin on 2022/1/7.
//
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include "sensorFusion.h"
#include "iofile.h"

int main(){

    std::string input_file_name = "../data/data-1.txt"; //sample-laser-radar-measurement-data-2.txt // data-1.txt
    std::string output_file_name = "../output/out-1.txt";
    std::string gt_file_name = "../output/gt-1.txt";

    // measurement_data：毫米波雷达/激光雷达实际测得的数据。数据包含测量值和时间戳，即融合算法的输入。
    // gt_data：每次测量时，障碍物位置的真值。对比融合算法输出和真值的差别，用于评估融合算法结果的好坏。
    MeasurementPackage measurement_data;
    GroundTruthPackage gt_data;

    if (!read_input_data(input_file_name, measurement_data, gt_data)){
        std::cout << "failed to open input data file" << std::endl;
        return -1;
    }
    std::cout << "Success to load data." << std::endl;

    // 部署跟踪算法
    SensorFusion fuser;

    std::vector<Eigen::Vector4d> output_data;

    int count = 0;
    for (size_t i = 0; i < measurement_data.meas_pack_list.size(); i++) {
//        if (count > 5) break;
        count++;
        fuser.Process(measurement_data.meas_pack_list[i]);
        Eigen::Vector4d x_out = fuser.kf_.GetX();

        output_data.push_back(x_out);
        std::cout << count << " x " << x_out(0) << ' ' << x_out(0) - gt_data.gt_pack_list[i].gt_values_(0)
                  << " y " << x_out(1) << ' ' << x_out(1) - gt_data.gt_pack_list[i].gt_values_(1)
                  << " vx " << x_out(2) << ' ' << x_out(2) - gt_data.gt_pack_list[i].gt_values_(2)
                  << " vy " << x_out(3) << ' ' << x_out(3) - gt_data.gt_pack_list[i].gt_values_(3)
                  << std::endl;

    }

    if (!write_output_data(output_file_name, output_data)){
        std::cout<<"Error: Could not write output file"<<std::endl;
        return -1;
    }

    if (!write_gt_data(gt_file_name, gt_data.gt_pack_list)){
        std::cout<<"Error: Could not write gt file"<<std::endl;
        return -1;
    }

    return 0;
}
