#include "ekf.h"
#include "ins_toolbox.h"

#include <iostream>
#include <fstream>
#define _USE_MATH_DEFINES
#include <cmath>
#include <yaml-cpp/yaml.h>


// EKF::EKF(const std::string &config): is_time_set(false), is_initialized(false), imu_flag(false), gps_flag(false), time_init(0)
EKF::EKF(const std::string &config_file): is_time_set(false), is_initialized(false), imu_flag(false), gps_flag(false), time_init(0)
{
    // Welcome message
    std::cout << "Hello EKF" << std::endl; 

    config = config_file;

    // Set variables to their default value & Set user configuration file
    Reset();

    // constant
    g << 0, 0, -9.80665;
    
}

void EKF::Reset()
{
        std::cout << "Reset" << std::endl;

        // Reset
        is_initialized = false;
        is_time_set = false;
        imu_flag = false;
        gps_flag = false; 

        time_init = 0;
        alignment_cnt = 0;

        g_body.setZero();
        pos_origin.setZero();
        LLH.setZero();
        Ce2n.setZero(); 
        sum_acc.setZero();
        sum_gyro.setZero();

        imu_time_now = 0;
        imu_time_prev = 0;
        imu_dt = 0;
        imu_acc.setZero();       
        imu_gyro.setZero();      

        gps_time_now = 0;
        gps_time_prev = 0;
        gps_dt = 0;
        gps_pos_ecef.setZero();
        gps_vel_ecef.setZero();

        P.setIdentity();
        Q.setIdentity();
        R.setIdentity();
        K.setZero();
        S.setZero();

        F.setZero();
        B.setZero();
        H.setZero();

        // set user configuration file
        LoadSettings(config);
}

void EKF::LoadSettings(const std::string &config)
{
    std::ifstream fin(config);
    if (!fin)
    {
        std::cerr << "Failed to open the file: " << config << std::endl;
        exit(-1);
    }
    
    YAML::Node cfg = YAML::LoadFile(config);

    // user settings
    alignment_time = cfg["alignment_time_sec"].as<double>();
    yaw_init = cfg["initial_yaw_deg"].as<double>();

    // covariance matrix
    P.block<3,3>(0,0) << Eigen::Matrix3d::Identity() * cfg["covP.pos"].as<double>();
    P.block<2,2>(3,3) << Eigen::Matrix2d::Identity() * cfg["covP.vel"].as<double>();
    P.block<1,1>(5,5) << cfg["covP.vel"].as<double>() * 1;
    P.block<1,1>(6,6) << cfg["covP.quat"].as<double>() * 10;
    P.block<2,2>(7,7) << Eigen::Matrix2d::Identity() * cfg["covP.quat"].as<double>();
    P.block<1,1>(9,9) << cfg["covP.quat"].as<double>() * 10;
    P.block<3,3>(10,10) << Eigen::Matrix3d::Identity() * cfg["covP.b_acc"].as<double>();
    P.block<3,3>(13,13) << Eigen::Matrix3d::Identity() * cfg["covP.b_gyro"].as<double>();
    
    Q.block<3,3>(0,0) << Eigen::Matrix3d::Identity() * cfg["covQ.n_acc"].as<double>();
    Q.block<3,3>(3,3) << Eigen::Matrix3d::Identity() * cfg["covQ.bi_acc"].as<double>();
    Q.block<3,3>(6,6) << Eigen::Matrix3d::Identity() * cfg["covQ.n_gyro"].as<double>();
    Q.block<3,3>(9,9) << Eigen::Matrix3d::Identity() * cfg["covQ.bi_gyro"].as<double>();

    R.block<3,3>(0,0) << Eigen::Matrix3d::Identity() * cfg["covR.pos"].as<double>();
    R.block<3,3>(3,3) << Eigen::Matrix3d::Identity() * cfg["covR.vel"].as<double>();

    std::cout << "Set config file: " << config << std::endl;
}

void EKF::SetTimeInit(const double &time)
{
    time_init = time;
    is_time_set = true;

    std::cout.precision(15);
    std::cout << "EKF started at: " << time_init << " sec"<< std::endl;
}

void EKF::SetYawInit(const float &yaw)
{
    // Reset all variables to restart the EKF
    Reset();

    // set yaw [deg]
    yaw_init = yaw;
}

void EKF::UpdateInputVector(const double &time, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro)
{
    imu_time_prev = imu_time_now;
    imu_time_now = time;
    imu_acc = acc;
    imu_gyro = gyro;
    imu_flag = true;
}

void EKF::UpdateMeasurementVector(const double &time, const Eigen::Vector3d &pos, const Eigen::Vector3d &vel)
{
    gps_time_prev = gps_time_now;
    gps_time_now = time;
    gps_pos_ecef = pos;
    gps_vel_ecef = vel;
    gps_flag = true;
}

void EKF::Alignment()
{
    Eigen::Vector3d pos_ned;
    Eigen::Vector3d vel_ned;
    Eigen::Quaterniond att_quat;
    Eigen::Vector3d bias_acc;
    Eigen::Vector3d bias_gyro;

    // Sum IMU data to estimate gyro bias and initial roll & pitch.
    if ((imu_time_now - time_init) < alignment_time)
    {
        alignment_cnt += 1;
        sum_gyro = sum_gyro + imu_gyro;
        sum_acc = sum_acc + imu_acc;
        imu_flag = false;   // reset imu flag

        return;
    }

    // Bias initialization
    bias_gyro = sum_gyro / alignment_cnt;
    bias_acc = Eigen::Vector3d::Zero();

    // Attitude initialization
    g_body = sum_acc / alignment_cnt;
    g_body = g_body / g_body.norm();

    Eigen::Vector3d g_navi = Eigen::Vector3d(0, 0, -1);
    Eigen::Vector3d g_mid = (g_body + g_navi) / (g_body + g_navi).norm();

    att_quat.w() = g_body.dot(g_mid);     // cos(angle/2)
    att_quat.vec() = g_body.cross(g_mid); // sin(angle/2)*rotation vector

    Eigen::Vector3d yaw = Eigen::Vector3d(0, 0, yaw_init*M_PI/180);
    att_quat = QuatProduct(Euler2Quat(yaw), att_quat);

    Eigen::Vector3d att_eul = Quat2Euler(att_quat);

    // Position initialization
    pos_origin << gps_pos_ecef.x(), gps_pos_ecef.y(), gps_pos_ecef.z();  // origin position (ECEF)

    LLH = ECEF2LLH(gps_pos_ecef);
    Ce2n << -sin(LLH.x())*cos(LLH.y()), -sin(LLH.x())*sin(LLH.y()), cos(LLH.x()),
            -sin(LLH.y()),              cos(LLH.y()),               0,
            -cos(LLH.x())*cos(LLH.y()), -cos(LLH.x())*sin(LLH.y()), -sin(LLH.x());

    pos_ned = Ce2n*(gps_pos_ecef-pos_origin);
    vel_ned = Ce2n*gps_vel_ecef;

    std::cout << "----- Alignment Finished-----" << std::endl;
    std::cout << "Initial position NED (m): " << pos_ned[0] << ", " << pos_ned[1] << ", " << pos_ned[2] << std::endl;
    std::cout << "Initial velocity NED (m/s): " << vel_ned[0] << ", " << vel_ned[1] << ", " << vel_ned[2] << std::endl; 
    std::cout << "Initial Attitude (deg): " << att_eul[0]*180/M_PI << ", " << att_eul[1]*180/M_PI << ", " << att_eul[2]*180/M_PI << std::endl;
    std::cout << std::endl;       

    // // Set Initial state
    nav_sol.SetNavSol(imu_time_now, pos_origin, pos_ned, vel_ned, att_quat, bias_acc, bias_gyro);
    nav_sol.SetImu(imu_acc, imu_gyro);

    // Update is_initialized
    is_initialized = true;
    imu_flag = false;   // reset imu flag

    return;
}

NavSol* EKF::Predict()
{
    Eigen::Vector3d pos_ned;
    Eigen::Vector3d vel_ned;
    Eigen::Quaterniond att_quat;
    Eigen::Vector3d bias_acc;
    Eigen::Vector3d bias_gyro;
    Eigen::Matrix<double, 3, 3> Cn2b; 
    Eigen::Matrix<double, 3, 3> Cb2n;

    double q0, q1, q2, q3;
    Eigen::Matrix<double, 3, 3> dCbn_dq1;
    Eigen::Matrix<double, 3, 3> dCbn_dq2;
    Eigen::Matrix<double, 3, 3> dCbn_dq3;
    Eigen::Matrix<double, 3, 3> dCbn_dq4;
    Eigen::Matrix<double, 4, 3> dQ_dq1;
    Eigen::Matrix<double, 4, 3> dQ_dq2;
    Eigen::Matrix<double, 4, 3> dQ_dq3;
    Eigen::Matrix<double, 4, 3> dQ_dq4;

    // load posteriori state
    Eigen::Matrix<double, 16, 1> x = nav_sol.GetNavSolState();

    pos_ned = x.block<3,1>(0,0);
    vel_ned = x.block<3,1>(3,0);
    att_quat.w() = x(6,0);
    att_quat.vec() << x.block<3,1>(7,0);
    bias_acc = x.block<3,1>(10,0);
    bias_gyro = x.block<3,1>(13,0);
    Cn2b = Quat2DCM(att_quat);
    Cb2n = Cn2b.transpose();

    // load imu
    imu_dt = imu_time_now - imu_time_prev;
    imu_flag = false;                        // reset imu flag

    // F matrix
    q0 = att_quat.w();
    q1 = att_quat.x();
    q2 = att_quat.y();
    q3 = att_quat.z();

    dCbn_dq1 << 2*q0, -2*q3, 2*q2, 2*q3, 2*q0, -2*q1, -2*q2, 2*q1, 2*q0;
    dCbn_dq2 << 2*q1, 2*q2, 2*q3, 2*q2, -2*q1, -2*q0, 2*q3, 2*q0, -2*q1;
    dCbn_dq3 << -2*q2, 2*q1, 2*q0, 2*q1, 2*q2, 2*q3, -2*q0, 2*q3, -2*q2;
    dCbn_dq4 << -2*q3, -2*q0, 2*q1, 2*q0, -2*q3, 2*q2, 2*q1, 2*q2, 2*q3;

    dQ_dq1 <<  0,  0,  0, 1,  0, 0, 0, 1,  0,  0, 0, 1;
    dQ_dq2 << -1,  0,  0, 0,  0, 0, 0, 0, -1,  0, 1, 0;
    dQ_dq3 <<  0, -1,  0, 0,  0, 1, 0, 0,  0, -1, 0, 0;
    dQ_dq4 <<  0,  0, -1, 0, -1, 0, 1, 0,  0,  0, 0, 0;

    F.setZero();
    F.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    F.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * imu_dt;
    F.block<3,1>(0,6) = dCbn_dq1 * (imu_acc - bias_acc) * 0.5 * imu_dt * imu_dt;
    F.block<3,1>(0,7) = dCbn_dq2 * (imu_acc - bias_acc) * 0.5 * imu_dt * imu_dt;
    F.block<3,1>(0,8) = dCbn_dq3 * (imu_acc - bias_acc) * 0.5 * imu_dt * imu_dt;
    F.block<3,1>(0,9) = dCbn_dq4 * (imu_acc - bias_acc) * 0.5 * imu_dt * imu_dt;
    F.block<3,3>(0,10) = -Cb2n * 0.5 * imu_dt * imu_dt;
    F.block<3,3>(3,3) = Eigen::Matrix3d::Identity();
    F.block<3,1>(3,6) = dCbn_dq1 * (imu_acc - bias_acc) * imu_dt;
    F.block<3,1>(3,7) = dCbn_dq2 * (imu_acc - bias_acc) * imu_dt;
    F.block<3,1>(3,8) = dCbn_dq3 * (imu_acc - bias_acc) * imu_dt;
    F.block<3,1>(3,9) = dCbn_dq4 * (imu_acc - bias_acc) * imu_dt;
    F.block<3,3>(3,10) = -Cb2n * imu_dt;
    F.block<4,1>(6,6) = dQ_dq1 * (imu_gyro - bias_gyro) * 0.5 * imu_dt;
    F.block<4,1>(6,7) = dQ_dq2 * (imu_gyro - bias_gyro) * 0.5 * imu_dt;
    F.block<4,1>(6,8) = dQ_dq3 * (imu_gyro - bias_gyro) * 0.5 * imu_dt;
    F.block<4,1>(6,9) = dQ_dq4 * (imu_gyro - bias_gyro) * 0.5 * imu_dt;
    F.block<4,4>(6,6) = F.block<4,4>(6,6) + Eigen::Matrix4d::Identity();
    F.block<4,3>(6,13) << -q1, -q2, -q3, q0, -q3, q2, q3, q0, -q1, -q2, q1, q0;
    F.block<4,3>(6,13) = F.block<4,3>(6,13) * (-1) * 0.5 * imu_dt;
    F.block<3,3>(10,10) = Eigen::Matrix3d::Identity();
    F.block<3,3>(13,13) = Eigen::Matrix3d::Identity();

    // B matrix
    B.setZero();
    B.block<3,3>(0,0) = F.block<3,3>(0,10);
    B.block<3,3>(3,0) = F.block<3,3>(3,10);
    B.block<4,3>(6,6) = F.block<4,3>(6,13);
    B.block<3,3>(10,3) = Eigen::Matrix3d::Identity();
    B.block<3,3>(13,9) = Eigen::Matrix3d::Identity();

    // error covariance propagation
    P = F*P*F.transpose() + B*Q*B.transpose();

    // update priori state
    pos_ned = pos_ned + vel_ned*imu_dt + (Cb2n*(imu_acc-bias_acc)-g)*imu_dt*imu_dt*0.5; 
    vel_ned = vel_ned + (Cb2n*(imu_acc-bias_acc)-g)*imu_dt;
    att_quat = QuatProduct(att_quat, Euler2Quat((imu_gyro-bias_gyro)*imu_dt));

    nav_sol.SetNavSol(imu_time_now, pos_origin, pos_ned, vel_ned, att_quat, bias_acc, bias_gyro);
    nav_sol.SetImu(imu_acc, imu_gyro);

    return &nav_sol;
}

NavSol* EKF::Correct()
{
    Eigen::Vector3d pos_ned;
    Eigen::Vector3d vel_ned;
    Eigen::Quaterniond att_quat;
    Eigen::Vector3d bias_acc;
    Eigen::Vector3d bias_gyro;
    Eigen::Matrix<double, 3, 3> Cn2b; 
    Eigen::Matrix<double, 3, 3> Cb2n;

    Eigen::Vector3d gps_pos_ned;
    Eigen::Vector3d gps_vel_ned;
    Eigen::Matrix<double,6,1> del_z;
    Eigen::Matrix<double,16,1> err_x;

    // load priori state
    Eigen::Matrix<double, 16, 1> x = nav_sol.GetNavSolState();
    pos_ned = x.block<3,1>(0,0);
    vel_ned = x.block<3,1>(3,0);
    att_quat.w() = x(6,0);
    att_quat.vec() << x.block<3,1>(7,0);
    bias_acc = x.block<3,1>(10,0);
    bias_gyro = x.block<3,1>(13,0);
    Cn2b = Quat2DCM(att_quat);
    Cb2n = Cn2b.transpose();

    // Measurement
    gps_dt = gps_time_now - gps_time_prev;

    // LLH = ECEF2LLH(gps_pos_ecef);
    LLH = ECEF2LLH(pos_origin);
    Ce2n << -sin(LLH.x())*cos(LLH.y()), -sin(LLH.x())*sin(LLH.y()), cos(LLH.x()),
            -sin(LLH.y()),              cos(LLH.y()),               0,
            -cos(LLH.x())*cos(LLH.y()), -cos(LLH.x())*sin(LLH.y()), -sin(LLH.x());

    gps_pos_ned = Ce2n*(gps_pos_ecef - pos_origin);
    gps_vel_ned = Ce2n*(gps_vel_ecef);
    gps_flag = false;                         // reset gps flag

    // H matrix
    H.setZero();
    H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    H.block<3,3>(3,3) = Eigen::Matrix3d::Identity();

    // Update Kalman gain & error covariance
    S = H*P*(H.transpose()) + R;
    K = P*(H.transpose())*(S.inverse());
    P = P - K*H*P;

    // error X
    del_z.block<3,1>(0,0) = gps_pos_ned - pos_ned;
    del_z.block<3,1>(3,0) = gps_vel_ned - vel_ned;
    err_x = Eigen::Matrix<double,16,1>::Zero() + K*(del_z - Eigen::Matrix<double,6,1>::Zero());

    // update posteriori state
    pos_ned = pos_ned + err_x.block<3,1>(0,0);
    vel_ned = vel_ned + err_x.block<3,1>(3,0);
    att_quat.w() = att_quat.w() + err_x(6,0);
    att_quat.x() = att_quat.x() + err_x(7,0);
    att_quat.y() = att_quat.y() + err_x(8,0);
    att_quat.z() = att_quat.z() + err_x(9,0);
    att_quat = att_quat.normalized();
    bias_acc = bias_acc + err_x.block<3,1>(10,0);
    bias_gyro = bias_gyro + err_x.block<3,1>(13,0);

    nav_sol.SetNavSol(gps_time_now, pos_origin, pos_ned, vel_ned, att_quat, bias_acc, bias_gyro);

    return &nav_sol;

}

void EKF::Run()
{
    while(true)
    {
        if (is_initialized)
        {
            if (imu_flag)
            {
                Predict();
            }
            if (gps_flag)
            {
                Correct();
            }
        }
        else
        {
            // if (imu_flag && gps_flag)
            if (imu_flag)
            {
                Alignment();
            }
        }
    }
}

void NavSol::SetNavSol(double &t, Eigen::Vector3d &orig, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Quaterniond &att, Eigen::Vector3d &ba, Eigen::Vector3d &bg)
{
    timestamp = t;
    origin = orig;
    position = pos;
    velocity = vel;
    attitude = att;
    bias_acc = ba;
    bias_gyro = bg; 
}

void NavSol::SetImu(Eigen::Vector3d &imu_acc, Eigen::Vector3d &imu_gyro)
{
    acceleration = imu_acc;
    angular_velocity = imu_gyro;
}

Eigen::Matrix<double, 16, 1> NavSol::GetNavSolState() const
{
    Eigen::Matrix<double, 16, 1> x;
    x << position, velocity, attitude.w(), attitude.vec(), bias_acc, bias_gyro;

    return x;
}

Eigen::Matrix<double, 20, 1> NavSol::GetNavRosMsg() const
{
    Eigen::Matrix<double, 20, 1> x;
    x << timestamp, origin, position, velocity, attitude.w(), attitude.vec(), acceleration, angular_velocity;

    return x;
}
