

#include "vm_basic_command.h"

void Vm_Basic_Command::EKF_Filter(void) {
    ros::Time after_filter;
    after_filter = ros::Time::now();
    ros::Duration duration = after_filter - filter_begin;
    double dt = duration.toSec();

    A_EKF << 1.0f, 0.0f, 0.0f, dt, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f, dt, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f, 0.0f, dt,
        0.0f, 0.0f, 0.0f, 1.0, 0.0, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0, 1.0, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f;
    Xp_EKF(0, 0) = x + dt * x_vel_measure + 0.5 * acc_x * dt * dt;
    Xp_EKF(1, 0) = y + dt * y_vel_measure + 0.5 * acc_y * dt * dt;
    Xp_EKF(2, 0) = z + z_vel * dt + acc_z * 0.5 * dt * dt;
    Xp_EKF(3, 0) = x_vel_measure + acc_x * dt;
    Xp_EKF(4, 0) = y_vel_measure + acc_y * dt;
    Xp_EKF(5, 0) = z_vel_measure + acc_z * dt;
    filter_begin = ros::Time::now();

    Pp_EKF = A_EKF * P_EKF * A_EKF.transpose() + Q_EKF;

    Eigen::MatrixXf temp;

    H_EKF << cos(-th_abs), -sin(-th_abs), 0, 0, 0, 0,
        sin(-th_abs), cos(-th_abs), 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, cos(-th_abs), -sin(-th_abs), 0,
        0, 0, 0, sin(-th_abs), cos(-th_abs), 0,
        0, 0, 0, 0, 0, 1;

    temp = H_EKF * Pp_EKF * H_EKF.transpose() + R_EKF;
    K_EKF = Pp_EKF * H_EKF.transpose() * temp.inverse();

    X_EKF = Xp_EKF + K_EKF * (Z_EKF - H_EKF * Xp_EKF);
    x = X_EKF(0, 0);
    if (!Marker_Landing_Working_) {
        y = X_EKF(1, 0);
        z = X_EKF(2, 0);
    }
    y = X_EKF(1, 0);
    z = X_EKF(2, 0);
    x_vel_measure = X_EKF(3, 0);
    y_vel_measure = X_EKF(4, 0);
    z_vel_measure = X_EKF(5, 0);

    P_EKF = Pp_EKF - K_EKF * H_EKF * Pp_EKF;

    filter_begin = ros::Time::now();
}

void Vm_Basic_Command::SigmaPoint(Eigen::MatrixXf &X, Eigen::MatrixXf &P, float kappa) {
    Xi = Eigen::MatrixXf::Zero(X_n, 2 * X_n + 1);
    W = Eigen::MatrixXf::Zero(2 * X_n + 1, 1);

    Xi.col(0) = X_UKF;

    W(0) = kappa / (X_n + kappa);

    Eigen::MatrixXf Temp;
    Temp = (X_n + kappa) * P_UKF;
    Eigen::MatrixXf L(Temp.llt().matrixL());

    for (int i = 0; i < X_n; i++) {
        Xi.col(i + 1) = X + L.row(i).transpose();
        W(i + 1) = 1 / (2 * (X_n + kappa));
        Xi.col(X_n + i + 1) = X - L.row(i).transpose();
        W(X_n + i + 1) = 1 / (2 * (X_n + kappa));
    }
}

void Vm_Basic_Command::UT(Eigen::MatrixXf &Xi, Eigen::MatrixXf &W, Eigen::MatrixXf &noiseCov, uint8_t type) {
    uint8_t n = Xi.rows();
    uint8_t kmax = Xi.cols();

    Eigen::MatrixXf Temp_X;
    Temp_X = Eigen::MatrixXf::Zero(6, 1);

    for (int i = 0; i < kmax; i++) {
        Temp_X = Temp_X + W(i) * Xi.col(i);
    }
    Eigen::MatrixXf xcov;
    xcov = Eigen::MatrixXf::Zero(n, n);
    for (int i = 0; i < kmax; i++) {
        xcov = xcov + W(i) * (Xi.col(i) - Temp_X) * (Xi.col(i) - Temp_X).transpose();
    }
    xcov = xcov + noiseCov;

    if (type == _UKF_X) {
        Xp_UKF = Temp_X;
        Pp_UKF = Eigen::MatrixXf::Zero(n, n);
        Pp_UKF = xcov;
    } else if (type == _UKF_Z) {
        Zp_UKF = Temp_X;
        Pz_UKF = Eigen::MatrixXf::Zero(n, n);
        Pz_UKF = xcov;
    }
}

Eigen::MatrixXf Vm_Basic_Command::CalFunction(Eigen::MatrixXf xhat) {
    Eigen::MatrixXf Xp(6, 1);
    //   Xp=Eigen::MatrixXf::Zero(6,1);
    ros::Time after_filter;
    after_filter = ros::Time::now();
    ros::Duration duration = after_filter - filter_begin;
    double dt = duration.toSec();
    Xp(0, 0) = xhat(0, 0) + dt * xhat(3, 0) + 0.5 * acc_x * dt * dt;
    Xp(1, 0) = xhat(1, 0) + dt * xhat(4, 0) + 0.5 * acc_y * dt * dt;
    Xp(2, 0) = xhat(2, 0) + xhat(5, 0) * dt + acc_z * 0.5 * dt * dt;
    Xp(3, 0) = xhat(3, 0) + acc_x * dt;
    Xp(4, 0) = xhat(4, 0) + acc_y * dt;
    Xp(5, 0) = xhat(5, 0) + acc_z * dt;

    // Xp_EKF(0, 0) = x + dt * x_vel_measure + 0.5 * acc_x * dt * dt;
    // Xp_EKF(1, 0) = y + dt * y_vel_measure + 0.5 * acc_y * dt * dt;
    Xp_EKF(2, 0) = z + z_vel * dt + acc_z * 0.5 * dt * dt;
    Xp_EKF(3, 0) = x_vel_measure + acc_x * dt;
    Xp_EKF(4, 0) = y_vel_measure + acc_y * dt;
    Xp_EKF(5, 0) = z_vel_measure + acc_z * dt;

    filter_begin = ros::Time::now();
    return Xp;
}

void Vm_Basic_Command::UKF_Filter(void) {
    SigmaPoint(X_UKF, P_UKF, 2);

    Eigen::MatrixXf FXi;
    FXi = Eigen::MatrixXf::Zero(X_n, 2 * X_n + 1);

    for (int i = 0; i < 2 * X_n + 1; i++) {
        FXi.col(i) = CalFunction(Xi.col(i));
    }

    UT(FXi, W, Q_UKF, _UKF_X);

    Eigen::MatrixXf HXi;
    HXi = Eigen::MatrixXf::Zero(X_m, 2 * X_n + 1);
    HXi = FXi;

    UT(HXi, W, R_UKF, _UKF_Z);

    Eigen::MatrixXf Pxz;
    Pxz = Eigen::MatrixXf::Zero(X_n, X_m);

    for (int i = 0; i < 2 * X_n + 1; i++) {
        Pxz = Pxz + W(i) * (Xi.col(i) - Xp_UKF) * (HXi.col(i) - Zp_UKF).transpose();
    }

    K_UKF = Pxz * Pz_UKF.inverse();
    X_UKF = Xp_UKF + K_UKF * (Z_UKF - Zp_UKF);
    P_UKF = Pp_UKF - K_UKF * Pz_UKF * K_UKF.transpose();

    x = X_UKF(0);
    y = X_UKF(1);
    z = X_UKF(2);
}

