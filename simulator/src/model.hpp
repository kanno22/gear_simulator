#ifndef _MODEL_H_
#define _MODEL_H_
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <iomanip>
#include <cmath>
#include "linkconf_ver2.hpp"    

// #define THETA_1_STANCE 0
// #define THETA_2_STANCE 1
// #define THETA_M_STANCE 2

// #define THETA_M 4

// #define K_G    8E2
class jointClass{
    public:
    double x;
    double z;
};

class stateClass{
    public:
    stateClass();
    
    Eigen::Matrix<double, 5, 1> pose;
    Eigen::Matrix<double, 5, 1> velo;
    Eigen::Matrix<double, 5, 1> accel;
    // Eigen::Matrix<double, 5, 1> jerk;
    double *x, *z, *theta_r, *theta_j, *theta_m;
    double *x_dot, *z_dot, *theta_r_dot, *theta_j_dot, *theta_m_dot;
    double wheel_velo_f,wheel_velo_f_x, wheel_velo_m, wheel_velo_m_x,wheel_velo_b,wheel_velo_b_x; 
    double external_forces[5];
    
    // double Tx;
    // double Tz;
    // double Tth;
    // double Ug;
    // double Uk;
    // double Energy;
    double springTorque;
    Eigen::Matrix<double, 3, 1> torque;
    // double d_torque;
    double wheel_h_min;
    jointClass joint[5];
    // jointClass cent_grav;
    // jointClass cent_grav_velo;
    void reset_state();
    void kinematics();
   
    void get_wheel_h_min();
   
};

class modelClass{
    public:
        modelClass();
        constexpr static double g = G_ACCEL;
        static Eigen::Matrix<double, 5, 1> tau;
        static Eigen::Matrix<double, 5, 5> inertia;
        static Eigen::Matrix<double, 5, 1> corioli_cent;
        static Eigen::Matrix<double, 5, 1> potential;
        static Eigen::Matrix<double, 5, 1> d_term;   
        static stateClass eq_differential(stateClass state);

};

#endif