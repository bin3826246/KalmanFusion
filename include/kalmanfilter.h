//
// Created by Duan Bin on 2022/1/7.
//

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Dense>
#include <cmath>

class KalmanFilter{
public:
    KalmanFilter();
    ~KalmanFilter();

    void Initialization(Eigen::VectorXd x_in);

    bool IsInitialized();

    void SetF(Eigen::MatrixXd F_in);

    void SetP(Eigen::MatrixXd P_in);

    void SetQ(Eigen::MatrixXd Q_in);

    void SetH(Eigen::MatrixXd H_in);

    void SetR(Eigen::MatrixXd R_in);

    void Prediction();

    void KFUpdate(Eigen::VectorXd z);

    void EKFUpdate(Eigen::VectorXd z);

    Eigen::VectorXd GetX();

private:

    void CalculateJacobianMatrix();

    // flag of initialization
    bool is_initialized_;

    // state vector
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // state transistion matrix
    Eigen::MatrixXd F_;

    // process covariance matrix
    Eigen::MatrixXd Q_;

    // measurement matrix
    Eigen::MatrixXd H_;

    // measurement covariance matrix
    Eigen::MatrixXd R_;

};

#endif //KALMANFILTER_H
