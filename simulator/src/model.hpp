#ifndef _MODEL_H_
#define _MODEL_H_
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <iomanip>
#include <cmath>
#include "linkconf_ver2.hpp"    


// ロボットの初期位置 
// x座標[m]
#define X_START -1
// z座標[m]
//#define Z_START 0.2
#define Z_START WHEEL_R 
// 初期角度[deg]
#define THETA_1_START 45
#define THETA_m_START 0
#define THETA_2_START 45
#define THETA_3_START -45

class jointClass{
    public:
    double x;
    double z;
};

class stateClass{
    public:
    stateClass();
    
    Eigen::Matrix<double, 6, 1> pose;
    Eigen::Matrix<double, 6, 1> velo;
    Eigen::Matrix<double, 6, 1> accel;
   
    double *x, *z, *theta_1, *theta_m, *theta_2, *theta_3;
    double *x_dot, *z_dot, *theta_1_dot, *theta_m_dot, *theta_2_dot, *theta_3_dot;
    double wheel_velo; 
    double theta_g;//重心角度
    //double dtheta_g;//重心角速度
    double external_forces[6];
    
    double springTorque;
    double torque;

    double wheel_h_min;
    jointClass joint[9];//+5

    void reset_state();
    void kinematics();
   
    void get_wheel_h_min();
};

class modelClass{
    public:
        modelClass();
        constexpr static double g = G_ACCEL;
        static Eigen::Matrix<double, 6, 1> tau;
        static Eigen::Matrix<double, 6, 6> inertia;
        static Eigen::Matrix<double, 6, 1> corioli_cent;
        static Eigen::Matrix<double, 6, 1> potential;
        static Eigen::Matrix<double, 6, 1> d_term;   
        static stateClass eq_differential(stateClass state);

};

#endif