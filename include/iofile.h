//
// Created by Duan Bin on 2022/7/11.
//

#ifndef KALMANFUSION_IOFILE_H
#define KALMANFUSION_IOFILE_H

#include "measurement_package.h"
#include "ground_truth_package.h"

/**
 * Reads input data from a file.
 * @param input_file_name Name of file containing input data.
 * @param meas Data of measurement sensor
 * @param gt Data of groundtruth
 * @output True if opening and reading file was successful
 */
inline bool read_input_data(const std::string& input_file_name, MeasurementPackage& meas, GroundTruthPackage& gt) {
    // Get file of map
    std::ifstream input_file(input_file_name.c_str(), std::ifstream::in);
    // Return if we can't open the file
    if (!input_file) {
        return false;
    }

    // 通过while循环将雷达测量值和真值全部读入内存，存入meas_pack_list和gt_pack_list中
    std::string line;
    while (std::getline(input_file, line)){
        std::string sensor_type;
        MeasurementPackage meas_temp;
        GroundTruthPackage gt_temp;
        std::istringstream iss(line);
        long long timestamp;

        // 读取当前行的第一个元素，L代表Lidar数据，R代表Radar数据
        iss >> sensor_type;
        if (sensor_type == "L"){
//            continue;
            // 激光雷达数据 Lidar data
            // 该行第二个元素为测量值x，第三个元素为测量值y，第四个元素为时间戳(纳秒）
            meas_temp.sensor_type_ = MeasurementPackage::LASER;
            meas_temp.raw_measurements_ = Eigen::VectorXd(2);
            double x;
            double y;
            iss >> x;
            iss >> y;
            meas_temp.raw_measurements_ << x, y;
            iss >> timestamp;
//            std::cout<<meas_temp.raw_measurements_[0]<<' '<<x<<std::endl;
            meas_temp.timestamp_ = timestamp;
            meas.meas_pack_list.push_back(meas_temp);
        } else if (sensor_type == "R"){
//            continue;
            // 毫米波雷达数据 Radar data
            // 该行第二个元素为距离pho，第三个元素为角度phi，第四个元素为径向速度pho_dot，第五个元素为时间戳(纳秒）
            meas_temp.sensor_type_ = MeasurementPackage::RADAR;
            meas_temp.raw_measurements_ = Eigen::VectorXd(3);
            double rho;
            double phi;
            double rho_dot;
            iss >> rho;
            iss >> phi;
            iss >> rho_dot;
            meas_temp.raw_measurements_ << rho, phi, rho_dot;
            iss >> timestamp;
            meas_temp.timestamp_ = timestamp;
            meas.meas_pack_list.push_back(meas_temp);
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
        gt_temp.gt_values_ = Eigen::VectorXd(4);
        gt_temp.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
        gt.gt_pack_list.push_back(gt_temp);
    }

    input_file.close();
    return true;
}

inline bool write_output_data(const std::string& output_file_name, std::vector<Eigen::Vector4d>& x_out_list){

    std::ofstream output_file (output_file_name, std::ofstream::out);

    if(!output_file){
        return false;
    }

    for (int i = 0; i < x_out_list.size(); i++){

        output_file << x_out_list[i](0) << '\t';
        output_file << x_out_list[i](1) << '\t';
        output_file << x_out_list[i](2) << '\t';
        output_file << x_out_list[i](3) << '\n';

    }
    output_file.close();

    return true;
}

inline bool write_gt_data(const std::string& gt_fileName, std::vector<GroundTruthPackage>& gt_data){

    std::ofstream gt_file (gt_fileName, std::ofstream::out);

    if(!gt_file){
        return false;
    }

    for (auto & gt : gt_data){
//        std::ostringstream oss_pos()

        gt_file << gt.gt_values_(0) << '\t';
        gt_file << gt.gt_values_(1) << '\t';
        gt_file << gt.gt_values_(2) << '\t';
        gt_file << gt.gt_values_(3) << '\n';
    }
    gt_file.close();

    return true;
}

#endif //KALMANFUSION_IOFILE_H
