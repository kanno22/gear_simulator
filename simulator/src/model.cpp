#include "model.hpp"  

// ロボットの初期位置 
// x座標[m]
#define X_START 0
// z座標[m]
#define Z_START WHEEL_R 
// 初期角度[deg]
#define THETA_1_START 0
#define THETA_m_START 0
#define THETA_2_START 0
#define THETA_3_START 0

extern double delta_t;

Eigen::Matrix<double, 6, 1> modelClass::tau;//外力項
Eigen::Matrix<double, 6, 6> modelClass::inertia;//慣性項
Eigen::Matrix<double, 6, 1> modelClass::corioli_cent;//速度項
Eigen::Matrix<double, 6, 1> modelClass::potential; //重力項
Eigen::Matrix<double, 6, 1> modelClass::d_term;//粘性項

stateClass::stateClass(){
    torque =0;
    x = &pose[0];
    z = &pose[1];
    theta_1 = &pose[2];
    theta_m = &pose[3];
    theta_2 = &pose[4];
    theta_3 = &pose[5];
    x_dot = &velo[0];
    z_dot = &velo[1];
    theta_1_dot = &velo[2];
    theta_m_dot = &velo[3];
    theta_2_dot = &velo[4];
    theta_3_dot = &velo[5];
    
    reset_state();
}

void stateClass::reset_state()
{

    pose << X_START, Z_START, M_PI*THETA_1_START/180, M_PI*THETA_m_START/180,THETA_2_START*M_PI/180,THETA_3_START*M_PI/180;
    velo << 0, 0, 0, 0, 0, 0;
    torque=0;
    
    kinematics();
}

void stateClass::kinematics()
{
    //足首関節
    joint[0].x = *x;
    joint[0].z = *z;

    // 1_m間関節
    joint[1].x = *x + L_1 * cos(*theta_1);
    joint[1].z = *z + L_1 * sin(*theta_1);
    
    //2_3間関節
    joint[2].x = *x+  L_1 * cos(*theta_1)+L_2 * cos(*theta_1+*theta_m+*theta_2);
    joint[2].z = *z + L_1 * sin(*theta_1)+L_2 * sin(*theta_1+*theta_m+*theta_2);
}

void stateClass::get_wheel_h_min()
{
    kinematics();
    wheel_h_min =  joint[0].z-WHEEL_R;

}

modelClass::modelClass()
{
    tau(0) = 0;
    tau(1) = 0;
    tau(2) = 0;
    tau(3) = 0;
    tau(4) = 0;
    tau(5) = 0;
}
    
stateClass modelClass::eq_differential(stateClass state)
{
    stateClass state_diff;
    double x,z,theta_1,theta_m,theta_2,theta_3;
    double dx, dz, dtheta_1, dtheta_m,dtheta_2,dtheta_3;
    x = state.pose(0);
    z = state.pose(1);
    theta_1 = state.pose(2);
    theta_m = state.pose(3);
    theta_2 = state.pose(4);
    theta_3 = state.pose(5);

    dx = state.velo(0);
    dz = state.velo(1);
    dtheta_1 = state.velo(2);
    dtheta_m = state.velo(3);
    dtheta_2 = state.velo(4);
    dtheta_3 = state.velo(5);

    inertia(0,0) = M_1 + M_2 + M_m + M_3;
    inertia(0,1) = 0;
    inertia(0,2) = -((M_1*P_1+(M_2+M_3+M_m)*L_1)*sin(theta_1)+(M_2*P_2+M_3*L_2)*sin(theta_1+theta_2+theta_m)+M_3*P_3*sin(theta_1+theta_2+theta_3+theta_m));
    inertia(0,3) = -((M_2*P_2+M_3*L_2)*sin(theta_1+theta_2+theta_m)+M_3*P_3*sin(theta_1+theta_2+theta_3+theta_m));
    inertia(0,4) = -((M_2*P_2+M_3*L_2)*sin(theta_1+theta_2+theta_m)+M_3*P_3*sin(theta_1+theta_2+theta_3+theta_m));
    inertia(0,5) = -M_3*P_3*sin(theta_1+theta_2+theta_3+theta_m);
    inertia(1,0) = 0;
    inertia(1,1) = M_1 + M_2 + M_m + M_3;
    inertia(1,2) = (M_1*P_1+(M_2+M_3+M_m)*L_1)*cos(theta_1)+(M_2*P_2+M_3*L_2)*cos(theta_1+theta_2+theta_m)+M_3*P_3*cos(theta_1+theta_2+theta_3+theta_m);
    inertia(1,3) = (M_2*P_2+M_3*L_2)*cos(theta_1+theta_2+theta_m)+M_3*P_3*cos(theta_1+theta_2+theta_3+theta_m);
    inertia(1,4) = (M_2*P_2+M_3*L_2)*cos(theta_1+theta_2+theta_m)+M_3*P_3*cos(theta_1+theta_2+theta_3+theta_m);
    inertia(1,5) = M_3*P_3*cos(theta_1+theta_2+theta_3+theta_m);
    inertia(2,0) = -((M_1*P_1+(M_2+M_3+M_m)*L_1)*sin(theta_1)+(M_2*P_2+M_3*L_2)*sin(theta_1+theta_2+theta_m)+M_3*P_3*sin(theta_1+theta_2+theta_3+theta_m));
    inertia(2,1) = (M_1*P_1+(M_2+M_3+M_m)*L_1)*cos(theta_1)+(M_2*P_2+M_3*L_2)*cos(theta_1+theta_2+theta_m)+M_3*P_3*cos(theta_1+theta_2+theta_3+theta_m);
    inertia(2,2) =  J_1+M_1*pow(P_1,2)+(M_2+M_3+M_m)*pow(L_1,2)+J_m+J_2+M_2*pow(P_2,2)+M_3*pow(L_2,2)+J_3+M_3*pow(P_3,2)+2*M_3*L_1*L_2*cos(theta_2+theta_m)+2*M_3*L_1*P_3*cos(theta_2+theta_3+theta_m)+2*M_3*L_2*P_3*cos(theta_3);
    inertia(2,3) =  J_m+J_2+M_2*pow(P_2,2)+M_3*pow(L_2,2)+J_3+M_3*pow(P_3,2)+M_3*L_1*L_2*cos(theta_2+theta_m)+M_3*L_1*P_3*cos(theta_2+theta_3+theta_m)+2*M_3*L_2*P_3*cos(theta_3);
    inertia(2,4) = J_2+M_2*pow(P_2,2)+M_3*pow(L_2,2)+J_3+M_3*pow(P_3,2)+M_3*L_1*L_2*cos(theta_2+theta_m)+M_3*L_1*P_3*cos(theta_2+theta_3+theta_m)+2*M_3*L_2*P_3*cos(theta_3);
    inertia(2,5) = J_3+M_3*pow(P_3,2)+M_3*L_1*P_3*cos(theta_2+theta_3+theta_m)+M_3*L_2*P_3*cos(theta_3);
    inertia(3,0) =  -(((M_2*P_2+M_3*L_2)*sin(theta_1+theta_2+theta_m)+M_3*P_3*sin(theta_1+theta_2+theta_3+theta_m)));
    inertia(3,1) =  (M_2*P_2+M_3*L_2)*cos(theta_1+theta_2+theta_m)+M_3*P_3*cos(theta_1+theta_2+theta_3+theta_m);
    inertia(3,2) =  J_m+J_2+M_2*pow(P_2,2)+M_3*pow(L_2,2)+J_3+M_3*pow(P_3,2)+M_3*L_1*L_2*cos(theta_2+theta_m)+M_3*L_1*P_3*cos(theta_2+theta_3+theta_m)+2*M_3*L_2*P_3*cos(theta_3);
    inertia(3,3) =  J_m+J_2+M_2*pow(P_2,2)+M_3*pow(L_2,2)+J_3+M_3*pow(P_3,2)+2*M_3*L_2*P_3*cos(theta_3);
    inertia(3,4) = J_2+M_2*pow(P_2,2)+M_3*pow(L_2,2)+J_3+M_3*pow(P_3,2)+2*M_3*L_2*P_3*cos(theta_3);
    inertia(3,5) = J_3+M_3*pow(P_3,2)+M_3*L_2*P_3*cos(theta_3);
    inertia(4,0) = -(((M_2*P_2+M_3*L_2)*sin(theta_1+theta_2+theta_m)+M_3*P_3*sin(theta_1+theta_2+theta_3+theta_m)));
    inertia(4,1) = (M_2*P_2+M_3*L_2)*cos(theta_1+theta_2+theta_m)+M_3*P_3*cos(theta_1+theta_2+theta_3+theta_m);
    inertia(4,2) = J_2+M_2*pow(P_2,2)+M_3*pow(L_2,2)+J_3+M_3*pow(P_3,2)+M_3*L_1*L_2*cos(theta_2+theta_m)+M_3*L_1*P_3*cos(theta_2+theta_3+theta_m)+2*M_3*L_2*P_3*cos(theta_3);
    inertia(4,3) = J_2+M_2*pow(P_2,2)+M_3*pow(L_2,2)+J_3+M_3*pow(P_3,2)+2*M_3*L_2*P_3*cos(theta_3);
    inertia(4,4) = J_2+M_2*pow(P_2,2)+M_3*pow(L_2,2)+J_3+M_3*pow(P_3,2)+2*M_3*L_2*P_3*cos(theta_3);
    inertia(4,5) = J_3+M_3*pow(P_3,2)+M_3*L_2*P_3*cos(theta_3);
    inertia(5,0) = -M_3*P_3*sin(theta_1+theta_2+theta_3+theta_m);
    inertia(5,1) = M_3*P_3*cos(theta_1+theta_2+theta_3+theta_m);
    inertia(5,2) = J_3+M_3*pow(P_3,2)+M_3*L_1*L_2*cos(theta_2+theta_m)+M_3*L_1*P_3*cos(theta_2+theta_3+theta_m)+M_3*L_2*P_3*cos(theta_3);
    inertia(5,3) = J_3+M_3*pow(P_3,2)+M_3*L_2*P_3*cos(theta_3);
    inertia(5,4) = J_3+M_3*pow(P_3,2)+M_3*L_2*P_3*cos(theta_3);
    inertia(5,5) = J_3+M_3*pow(P_3,2);

    corioli_cent(0) = -(M_1*P_1+(M_2+M_3+M_m)*L_1)*pow(dtheta_1,2)*cos(theta_1)-(M_2+M_3+M_m)*pow(dtheta_1+dtheta_2+dtheta_m,2)*cos(theta_1+theta_2+theta_m)-M_3*P_3*pow(dtheta_1+dtheta_2+dtheta_3+dtheta_m,2)*cos(theta_1+theta_2+theta_3+theta_m);
    corioli_cent(1) =  -(M_1*P_1+(M_2+M_3+M_m)*L_1)*pow(dtheta_1,2)*sin(theta_1)-(M_2+M_3+M_m)*pow(dtheta_1+dtheta_2+dtheta_m,2)*sin(theta_1+theta_2+theta_m)-M_3*P_3*pow(dtheta_1+dtheta_2+dtheta_3+dtheta_m,2)*sin(theta_1+theta_2+theta_3+theta_m);
    corioli_cent(2) =   -M_3*L_1*L_2*(dtheta_2+dtheta_m)*(2*dtheta_1+dtheta_2+dtheta_m)*sin(theta_2+theta_m)-M_3*L_1*P_3*(dtheta_2+dtheta_3+dtheta_m)*(2*dtheta_1+dtheta_2+dtheta_3+dtheta_m)*sin(theta_2+theta_3+theta_m)-M_3*L_2*P_3*dtheta_3*(2*dtheta_1+2*dtheta_2+dtheta_3+2*dtheta_m)*sin(theta_3);
    corioli_cent(3) =   -M_3*L_2*P_3*dtheta_3*(2*dtheta_1+2*dtheta_2+dtheta_3+2*dtheta_m)*sin(theta_3);
    corioli_cent(4) = -M_3*L_2*P_3*dtheta_3*(2*dtheta_1+2*dtheta_2+dtheta_3+2*dtheta_m)*sin(theta_3);
    corioli_cent(5) = M_3*L_2*P_3*pow(dtheta_1+dtheta_2+dtheta_m,2)*sin(theta_3);

    potential(0) =  0;
    potential(1) =  (M_1 + M_2 + M_m + M_3)*g;
    potential(2) =  (M_1*P_1+M_m*L_1)*g*cos(theta_1)+M_2*g*(L_1*cos(theta_1)+P_2*cos(theta_1+theta_2+theta_m))+M_3*g*(L_1*cos(theta_1)+L_2*cos(theta_1+theta_2+theta_m)+P_3*cos(theta_1+theta_2+theta_3+theta_m));
    potential(3) =  K_SPRING*theta_m+M_2*g*P_2*cos(theta_1+theta_2+theta_m)+M_3*g*(L_2*cos(theta_1+theta_2+theta_m)+P_3*cos(theta_1+theta_2+theta_3+theta_m));
    potential(4) =  M_2*g*P_2*cos(theta_1+theta_2+theta_m)+M_3*g*(L_2*cos(theta_1+theta_2+theta_m)+P_3*cos(theta_1+theta_2+theta_3+theta_m));
    potential(5) = M_3*g*P_3*cos(theta_1+theta_2+theta_3+theta_m);

    for(int i=0;i<6; i++) tau(i) = state.external_forces[i];
 
    d_term(0) = 0;
    d_term(1) = 0;
    d_term(2) = 0;
    d_term(3) = C_1_2*(dtheta_m+dtheta_2);
    d_term(4) = C_1_2*dtheta_m+(C_1_2+C_2)*dtheta_2;
    d_term(5) = 0;

    state_diff.pose = state.velo;
    state_diff.velo = inertia.inverse() * (tau - corioli_cent - potential -d_term);//運動方程式(加速度)

    return state_diff;
}

