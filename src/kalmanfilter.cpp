//
// Created by Duan Bin on 2022/1/7.
//

#include "kalmanfilter.h"

KalmanFilter::KalmanFilter() {

}

KalmanFilter::~KalmanFilter() {

}

void KalmanFilter::Initialization(Eigen::VectorXd x_in) {

}

bool KalmanFilter::IsInitialized() {
    return false;
}

void KalmanFilter::SetF(Eigen::MatrixXd F_in) {

}

void KalmanFilter::SetP(Eigen::MatrixXd P_in) {

}

void KalmanFilter::SetQ(Eigen::MatrixXd Q_in) {

}

void KalmanFilter::SetH(Eigen::MatrixXd H_in) {

}

void KalmanFilter::SetR(Eigen::MatrixXd R_in) {

}

void KalmanFilter::Prediction() {

}

void KalmanFilter::KFUpdate(Eigen::VectorXd z) {

}

void KalmanFilter::EKFUpdate(Eigen::VectorXd z) {

}

Eigen::VectorXd KalmanFilter::GetX() {
    return Eigen::VectorXd();
}
