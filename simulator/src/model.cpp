#include "model.hpp"  

// ロボットの初期位置 
// x座標[m]
#define X_START -(L_RGx+L_JBWx+WHEEL_R+0.02)
// z座標[m]
#define Z_START (L_RGWz + WHEEL_R) 
// ロッカーリンクの角度[deg]
#define THETA_Rocker_START 0
// 関節角度[deg]
#define THETA_Joint_START 0
// モータ角度[deg]
#define THETA_MOTOR_START 0

extern double delta_t;

Eigen::Matrix<double, 5, 1> modelClass::tau;//外力項
Eigen::Matrix<double, 5, 5> modelClass::inertia;//慣性項
Eigen::Matrix<double, 5, 1> modelClass::corioli_cent;//速度項
Eigen::Matrix<double, 5, 1> modelClass::potential; //重力項
Eigen::Matrix<double, 5, 1> modelClass::d_term;//粘性項

stateClass::stateClass(){
    torque << 0,0,0;
    x = &pose[0];
    z = &pose[1];
    theta_r = &pose[2];
    theta_j = &pose[3];
    theta_m = &pose[4];
    x_dot = &velo[0];
    z_dot = &velo[1];
    theta_r_dot = &velo[2];
    theta_j_dot = &velo[3];
    theta_m_dot = &velo[4];
    reset_state();
}

void stateClass::reset_state(){

    pose << X_START, Z_START, M_PI*THETA_Rocker_START/180, M_PI*THETA_Joint_START/180,THETA_MOTOR_START*M_PI/180;
    velo << 0, 0, 0, 0, 0;
    torque << 0,0,0;
    // d_torque = 0;
    kinematics();
}

void stateClass::kinematics(){
    //ロッカーリンクの重心位置
    joint[0].x = *x;
    joint[0].z = *z;

    // joint
    joint[1].x = *x + L_RGx * cos(*theta_r) + L_RGz * sin(*theta_r);
    joint[1].z = *z - L_RGz * cos(*theta_r) + L_RGx * sin(*theta_r);
    double joint_velo_x = *x_dot - L_RGx * sin(*theta_r) * (*theta_r_dot) + L_RGz * cos(*theta_r) * (*theta_r_dot);
    double joint_velo = *z_dot + L_RGz * sin(*theta_r) * (*theta_r_dot) + L_RGx * cos(*theta_r) * (*theta_r_dot);
    
    // back wheel
    joint[2].x = *x- L_RGWx * cos(*theta_r) + L_RGWz * sin(*theta_r);
    joint[2].z = *z- L_RGWz * cos(*theta_r) - L_RGWx * sin(*theta_r);
    wheel_velo_b_x = *x_dot+ L_RGWx * sin(*theta_r)* (*theta_r_dot) + L_RGWz*cos(*theta_r)* (*theta_r_dot);
    wheel_velo_b = *z_dot+ L_RGWz * sin(*theta_r)* (*theta_r_dot) - L_RGWx*cos(*theta_r)* (*theta_r_dot);
    
    // middle wheel
    joint[3].x = joint[1].x - L_JBWx * cos(*theta_r + *theta_j) + L_JBWz * sin(*theta_r + *theta_j);
    joint[3].z = joint[1].z - L_JBWz * cos(*theta_r + *theta_j) - L_JBWx * sin(*theta_r + *theta_j);
    wheel_velo_m_x = joint_velo_x + L_JBWx * sin(*theta_r + *theta_j)*(*theta_r_dot+*theta_j_dot) +L_JBWz*cos(*theta_r+*theta_j)*(*theta_r_dot+*theta_j_dot);
    wheel_velo_m = joint_velo + L_JBWz * sin(*theta_r + *theta_j)*(*theta_r_dot+*theta_j_dot) -L_JBWx*cos(*theta_r+*theta_j)*(*theta_r_dot+*theta_j_dot);
    
    // front wheel
    joint[4].x = joint[1].x + L_JBWx * cos(*theta_r + *theta_j) + L_JBWz * sin(*theta_r + *theta_j);
    joint[4].z = joint[1].z - L_JBWz * cos(*theta_r + *theta_j) + L_JBWx * sin(*theta_r + *theta_j);
    wheel_velo_f_x = joint_velo_x - L_JBWx * sin(*theta_r + *theta_j)*(*theta_r_dot+*theta_j_dot) +L_JBWz*cos(*theta_r+*theta_j)*(*theta_r_dot+*theta_j_dot);
    wheel_velo_f = joint_velo + L_JBWz * sin(*theta_r + *theta_j)*(*theta_r_dot+*theta_j_dot) +L_JBWx*cos(*theta_r+*theta_j)*(*theta_r_dot+*theta_j_dot);

}

modelClass::modelClass(){//外力項の初期化
    tau(0) = 0;
    tau(1) = 0;
    tau(2) = 0;
    tau(3) = 0;
}
    
stateClass modelClass::eq_differential(stateClass state){//運動方程式を記述

    //------------------------------------
    stateClass state_diff;
    double x,z,theta_r,theta_j,theta_m;
    double dx, dz, dtheta_r, dtheta_j,dtheta_m;
    x = state.pose(0);
    z = state.pose(1);
    theta_r = state.pose(2);
    theta_j = state.pose(3);
    theta_m = state.pose(4);
    dx = state.velo(0);
    dz = state.velo(1);
    dtheta_r = state.velo(2);
    dtheta_j = state.velo(3);
    dtheta_m = state.velo(4);
    
    double theta_ro = -atan(L_RGz/L_RGx);
    double theta_jo = -atan(L_BGz/L_BGx);
    double L_RG = sqrt(pow(L_RGx,2) + pow(L_RGz,2));
    double L_BG = sqrt(pow(L_BGx,2) + pow(L_BGz,2));

    inertia(0,0) = M_R + M_B;
    inertia(0,1) = 0;
    inertia(0,2) = -L_BG*M_B*sin(theta_ro+theta_r+theta_jo+theta_j)-L_RG*M_B*sin(theta_ro+theta_r);
    inertia(0,3) = -L_BG*M_B*sin(theta_ro+theta_r+theta_jo+theta_j);
    inertia(0,4) = 0;
    inertia(1,0) = 0;
    inertia(1,1) = M_R + M_B;
    inertia(1,2) = -L_BG*M_B*cos(theta_ro+theta_r+theta_jo+theta_j)-L_RG*M_B*cos(theta_ro+theta_r);
    inertia(1,3) = -L_BG*M_B*cos(theta_ro+theta_r+theta_jo+theta_j);
    inertia(1,4) = 0;
    inertia(2,0) = -L_BG*M_B*sin(theta_ro+theta_r+theta_jo+theta_j)-L_RG*M_B*sin(theta_ro+theta_r);
    inertia(2,1) = -L_BG*M_B*cos(theta_ro+theta_r+theta_jo+theta_j)-L_RG*M_B*cos(theta_ro+theta_r);
    inertia(2,2) =  2*L_BG*L_RG*M_B*cos(theta_jo+theta_j)+(pow(L_RG,2)+pow(L_BG,2))*M_B+J_R+J_B+J_M;
    inertia(2,3) =  L_BG*L_RG*M_B*cos(theta_jo+theta_j)+pow(L_BG,2)*M_B+J_B;
    inertia(2,4) = J_M;
    inertia(3,0) =  -L_BG*M_B*sin(theta_ro+theta_r+theta_jo+theta_j);
    inertia(3,1) =  -L_BG*M_B*cos(theta_ro+theta_r+theta_jo+theta_j);
    inertia(3,2) =  L_BG*L_RG*M_B*cos(theta_jo+theta_j)+pow(L_BG,2)*M_B+J_B;
    inertia(3,3) =  pow(L_BG,2)*M_B+J_B;
    inertia(3,4) = 0;
    inertia(4,0) = 0;
    inertia(4,1) = 0;
    inertia(4,2) = J_M;
    inertia(4,3) = 0;
    inertia(4,4) = J_M;

    corioli_cent(0) = (-L_BG*M_B*pow(dtheta_r,2)-2*L_BG*M_B*(dtheta_j)*(dtheta_r)-L_BG*M_B*pow(dtheta_j,2))*cos(theta_ro+theta_r+theta_jo+theta_j)
                        -L_RG*M_B*pow(dtheta_r,2)*cos(theta_ro+theta_r);
    corioli_cent(1) =  (L_BG*M_B*pow(dtheta_r,2)+2*L_BG*M_B*(dtheta_j)*(dtheta_r)+L_BG*M_B*pow(dtheta_j,2))*sin(theta_ro+theta_r+theta_jo+theta_j)
                        +L_RG*M_B*pow(dtheta_r,2)*sin(theta_ro+theta_r);
    corioli_cent(2) =   -2*L_BG*L_RG*M_B*(dtheta_j)*sin(theta_jo+theta_j)*(dtheta_r)-L_BG*L_RG*M_B*pow(dtheta_j,2)*sin(theta_jo+theta_j);
    corioli_cent(3) =   L_BG*L_RG*M_B*sin(theta_jo+theta_j)*pow(dtheta_r,2);
    corioli_cent(4) = 0;

    potential(0) =  0;
    potential(1) =  -M_R*g-M_B*g;
    potential(2) =  -M_B*g*(-L_BG*cos(theta_ro+theta_r+theta_jo+theta_j)-L_RG*cos(theta_ro+theta_r));

    potential(3) =  -L_BGz*M_B*g*sin(theta_r+theta_j)
                    +(K_SPRING*L_M*L_RJ*(sqrt(2*L_M*L_RJ*cos(theta_m-theta_j)+pow(L_RJ,2)+pow(L_M,2))-L_KN)*sin(theta_m-theta_j))
                    /sqrt(2*L_M*L_RJ*cos(theta_m-theta_j)+pow(L_RJ,2)+pow(L_M,2));

    potential(4) =  -(K_SPRING*L_M*L_RJ*(sqrt(2*L_M*L_RJ*cos(theta_m-theta_j)+pow(L_RJ,2)+pow(L_M,2))-L_KN)*sin(theta_m-theta_j))/sqrt(2*L_M*L_RJ*cos(theta_m-theta_j)+pow(L_RJ,2)+pow(L_M,2));

    // double potential_gravity = L_BG*M_B*g*cos(theta_ro+theta_r+theta_jo+theta_j);
    // double potential_spring = +(K_SPRING*L_C*L_RJ*(sqrt(-2*L_C*L_RJ*cos(theta_j-THETA_JO)
    //                 +pow(L_RJ,2)+pow(L_C,2))-L_KN)*sin(theta_j-THETA_JO))/(sqrt(-2*L_C*L_RJ*cos(theta_j-THETA_JO)+pow(L_RJ,2)+pow(L_C,2)));

    for(int i=0;i<5; i++) tau(i) = state.external_forces[i];//外力項
    double max_torque = 5.* 1.46 *(6110.-5.*dtheta_m/2./M_PI*60.)/6110.;
    if(max_torque<0) max_torque =0;
    if(tau[4]>max_torque) tau[4] = max_torque;

    d_term(0) = 0;
    d_term(1) = 0;
    d_term(2) = 0;
    d_term(3) = C_JOINT_FRICTION*dtheta_j;
    d_term(4) = C_MOTOR_FRICTION*(dtheta_m);

    state_diff.pose = state.velo;//(速度)
    state_diff.velo = inertia.inverse() * (tau - corioli_cent + potential -d_term);//運動方程式(加速度)

    return state_diff;
}

void stateClass::get_wheel_h_min()
{
    kinematics();
    wheel_h_min =  joint[2].z-WHEEL_R;//まず後輪の高さを算出
  
    for(int i=0; i<2; i++)
    {
        if( wheel_h_min >  joint[i+3].z-WHEEL_R)
        {
        wheel_h_min =  joint[i+3].z-WHEEL_R;//最も低い車輪高さを算出
        //最も低い車輪が地面に触れているとする
        }
    }
}
