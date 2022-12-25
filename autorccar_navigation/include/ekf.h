#ifndef EKF_H
#define EKF_H

#include <eigen3/Eigen/Dense>
#include <string>

class NavSol
{
    private:
        double timestamp;
        Eigen::Vector3d origin;           // origin position in ECEF [m]
        Eigen::Vector3d position;         // position in NED [m]
        Eigen::Vector3d velocity;         // velocity in NED [m/s]
        Eigen::Quaterniond attitude;      // attitude quaternion
        Eigen::Vector3d bias_acc;         // accelerometer bias
        Eigen::Vector3d bias_gyro;        // gyroscope bias
        Eigen::Vector3d acceleration;     // acceleration [m/s^2]
        Eigen::Vector3d angular_velocity; // angular velocity [rad/s]
    public:
        void SetNavSol(double &t, Eigen::Vector3d &orig, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Quaterniond &att, Eigen::Vector3d &ba, Eigen::Vector3d &bg);
        void SetImu(Eigen::Vector3d &imu_acc, Eigen::Vector3d &imu_gyro);
        Eigen::Matrix<double, 16, 1> GetNavSolState() const;
        Eigen::Matrix<double, 20, 1> GetNavRosMsg() const; 
};


class EKF
{
    public:
        EKF(const std::string & config);

        bool is_time_set;
        bool is_initialized;
        
        bool imu_flag;
        bool gps_flag;

        void SetTimeInit(const double &time);
        void SetYawInit(const float &yaw);

        void UpdateInputVector(const double &time, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro);
        void UpdateMeasurementVector(const double &time, const Eigen::Vector3d &pos, const Eigen::Vector3d &vel);

        void Alignment();
        NavSol* Predict();
        NavSol* Correct();
        
    private:
        double time_init;   // EKF start time, to check alignment 

        std::string config; // EKF config file

        // initialization variables
        int alignment_cnt;
        double alignment_time;          // [sec]
        double yaw_init;                // [deg]
        Eigen::Vector3d g;              // gravity 
        Eigen::Vector3d g_body;         // gravity in body frame
        Eigen::Vector3d pos_origin;
        Eigen::Vector3d LLH;
        Eigen::Matrix<double, 3, 3> Ce2n; // ECEF to NED
        Eigen::Vector3d sum_acc;
        Eigen::Vector3d sum_gyro;

        // IMU data
        double imu_time_now;
        double imu_time_prev;
        double imu_dt;
        Eigen::Vector3d imu_acc;       // IMU acc measurement [m/s^2]
        Eigen::Vector3d imu_gyro;      // IMU gyro measurement [rad/s]

        // GPS data
        double gps_time_now;
        double gps_time_prev;
        double gps_dt;
        Eigen::Vector3d gps_pos_ecef;  // GPS position - ECEF frame
        Eigen::Vector3d gps_vel_ecef;

        // filter output
        NavSol nav_sol;

        // covariance matrix
        Eigen::Matrix<double, 16, 16> P;
        Eigen::Matrix<double, 12, 12> Q;
        Eigen::Matrix<double, 6, 6> R;
        Eigen::Matrix<double, 16, 6> K;
        Eigen::Matrix<double, 6, 6> S;

        // transition matrix, control input matrix, observation matrix
        Eigen::Matrix<double, 16, 16> F;
        Eigen::Matrix<double, 16, 12> B;
        Eigen::Matrix<double, 6, 16> H;

        // set the variables to their default value
        void Reset();
        void LoadSettings(const std::string &config_file);

        void Run();

};

#endif