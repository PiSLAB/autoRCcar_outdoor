#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <vector>
#include <string>

#include <eigen3/Eigen/Dense>

#include "../include/ekf.h"

using namespace std;

void LoadIMUdata(const string &imu_filename, vector<vector<double>> &vIMUdata);
void LoadGPSdata(const string &gps_filename, vector<vector<double>> &vGPSdata);
void saveNavData(const string &output_filename, const Eigen::Matrix<double, 20, 1> &x);

int main()
{
    string imu_filename = "imu.csv";
    string gps_filename = "gps.csv";
    string output_filename = "output.csv";
    string config = "../config.yaml";

    if (remove(output_filename.c_str()))
    {
        cout << output_filename << " deleted."<< endl;
    }

    int gps_start_idx = 49;
    int imu_end_idx = 188531;

    vector<vector<double>> vIMUdata;  // time[sec], gyro[rad/s], acc[m/s^2]
    vector<vector<double>> vGPSdata;  // clock[sec], week[sec], pos_ecef[m], vel_ecef[m/s]

    // Load IMU & GPS data
    LoadIMUdata(imu_filename, vIMUdata);
    LoadGPSdata(gps_filename, vGPSdata);

    EKF nav(config);
    NavSol* mpSol;
    Eigen::Matrix<double, 20, 1> x;

    nav.SetTimeInit(vIMUdata[0][0]);

    // update GPS data for alignment
    double gps_time = vGPSdata[gps_start_idx-1][0];
    Eigen::Vector3d gps_pos = Eigen::Vector3d(vGPSdata[gps_start_idx-1][2], vGPSdata[gps_start_idx-1][3], vGPSdata[gps_start_idx-1][4]);
    Eigen::Vector3d gps_vel = Eigen::Vector3d(vGPSdata[gps_start_idx-1][5], vGPSdata[gps_start_idx-1][6], vGPSdata[gps_start_idx-1][7]);
    nav.UpdateMeasurementVector(gps_time, gps_pos, gps_vel);

    for (int i = 0; i < imu_end_idx+1; i++)
    {
        // Get IMU data
        double imu_time = vIMUdata[i][0];
        Eigen::Vector3d acc = Eigen::Vector3d(vIMUdata[i][4], vIMUdata[i][5], vIMUdata[i][6]);
        Eigen::Vector3d gyro = Eigen::Vector3d(vIMUdata[i][1], vIMUdata[i][2], vIMUdata[i][3]);
        nav.UpdateInputVector(imu_time, acc, gyro);

        // CORRECTION
        if (vIMUdata[i][0] > vGPSdata[gps_start_idx][0])
        {
            // Get GPS data
            double gps_time = vGPSdata[gps_start_idx][0];
            Eigen::Vector3d gps_pos = Eigen::Vector3d(vGPSdata[gps_start_idx][2], vGPSdata[gps_start_idx][3], vGPSdata[gps_start_idx][4]);
            Eigen::Vector3d gps_vel = Eigen::Vector3d(vGPSdata[gps_start_idx][5], vGPSdata[gps_start_idx][6], vGPSdata[gps_start_idx][7]);
            nav.UpdateMeasurementVector(gps_time, gps_pos, gps_vel);

            if (nav.is_initialized)
            {
                mpSol = nav.Correct();
                x = mpSol->GetNavRosMsg();
                saveNavData(output_filename, x);
            }
            gps_start_idx += 1;
        }

        // PROPAGATION
        if (nav.is_initialized)
        {
            mpSol = nav.Predict();

            x = mpSol->GetNavRosMsg();
            saveNavData(output_filename, x);
        }
        else
        {
            nav.Alignment();
        }
    }

    cout << "----- Done -----" << endl;

    return 0;
}

void LoadIMUdata(const string &imu_filename, vector<vector<double>> &vIMUdata)
{
    fstream f;
    f.open(imu_filename.c_str(), ios::in);
    
    while(!f.eof())
    {
        string s;
        
        getline(f, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            
            vector<double> tmp(7);
            int idx = 0;

            for (int i=0; i<8; i++)
            {
                string sss;
                getline(ss, sss, ',');
                double t = stod(sss);
                if (i==0){
                    t = t/1e3;        // imu clock[sec]
                }
                else if (i==1){ 
                    continue;
                }
                else if (i==5 || i==6 || i==7){
                    t = t * 9.80665;  // specific force [m/s^2]
                }
                
                tmp[idx] = t;
                idx += 1;
            }
            vIMUdata.push_back(tmp);
        }
    }
    f.close();
}

void LoadGPSdata(const string &gps_filename, vector<vector<double>> &vGPSdata)
{
    fstream f;
    f.open(gps_filename.c_str(), ios::in);

    while(!f.eof())
    {
        string s;
        
        getline(f, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            
            vector<double> tmp(8);
            int idx = 0;

            for (int i=0; i<16; i++)
            {
                string sss;
                getline(ss, sss, ',');
                double t = stod(sss);
                if (i==0){
                    t = t/1e3;  // gps clock [sec]
                }
                else if (i==2){ 
                    t = t/1e3;  // gps week [sec]
                }
                else if (i==1 || i==3 || i==4 || i==8 || i==9 || i==10 || i==11 || i==12){
                    continue;
                }
                tmp[idx] = t;
                idx += 1;
            }
            vGPSdata.push_back(tmp);
        }
    }
    f.close();
}

void saveNavData(const string &output_filename, const Eigen::Matrix<double, 20, 1> &x)
{
    ofstream f;

    int row = x.rows();

    f.open(output_filename, ios::app);
    f.precision(18);
    for (int i=0; i<row-1; i++)
    {
        f << x[i] << ", ";
    }
    f << x[row-1] << "\n";

    f.close();
}