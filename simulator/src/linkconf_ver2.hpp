#ifndef _LINKCONF_VER2_H_
#define _LINKCONF_VER2_H_
#include <cmath>


// -----変える必要有り ここから↓-----

//リンクの質量[kg]
#define M_1  0.13//下腿
#define M_m  0.05//モータ
#define M_2  0.56//大腿
#define M_3  0.2//ボディ
//#define M_3 0.5

//リンク長さ[m]
#define L_1  0.2//下腿
#define P_1  0.1
#define L_2  0.2//大腿
#define P_2  0.05
#define L_3  0.1//ボディ
#define P_3  0.05
// #define L_3 0.21
// #define P_3 0.21

//慣性モーメント[kg m^2]
#define	J_1		6.851E-3//下腿
#define	J_m		3.915776E-3//モータ
#define J_2     7.862E-3//大腿
#define J_3     5.0E-3 //ボディ
//#define J_3 0

// 車輪半径[m]
#define WHEEL_R 0.025

// 段差の水平位置[m]
#define STEP_X 0
// 段差の高さ[m]
#define STEP_HIGHT 0.0

// 最大車軸トルク[N m]
#define MAX_TORQUE 5

//最大モータリンクトルク[N m]
#define MAX_M_JOINT_TORQUE 5
//最大ボディリンクトルク[N m]
#define  MAX_B_JOINT_TORQUE 5
//最大車輪トルク[N m]
#define  MAX_WHEEL_TORQUE 5

// ばね定数[Nm/rad]
//#define K_SPRING 1.92
#define K_SPRING 1000

// ばねのオフセット角度
#define THETA_Jm (0*M_PI/180)

//関節の粘性係数
#define C_1_2 0.001
#define C_2 0.02
// #define C_MOTOR_FRICTION (0.000)

// 重力加速度
#define G_ACCEL 9.81

// ホイールの弾性係数、粘性係数
#define K_WHEEL 5000
#define C_WHEEL 100

//AngleControl用のゲイン 
#define P_GEIN 20
#define D_GEIN 2

// ディスプレイ設定
// 描画の大きさ
#define IMAGE_WIDTH  720.0
#define IMAGE_HEIGHT 480.0
#define RATIO_XY (IMAGE_WIDTH /IMAGE_HEIGHT) 
// ロボットの描画位置
#define Y_DISP -0.3//-0.7//-0.5
#define X_DISP 0
// ロボットの描画の大きさ
#define PLOT_RATE 1.5

#define X_WINDOWSIZE 600.0
#define Y_WINDOWSIZE 900.0

// -----必要に応じて変更 ここまで↑-----

// 変更不要
// #define L_RGW   (sqrt(L_RGWx*L_RGWx+L_RGWz*L_RGWz))
// #define L_RGJ   (sqrt(L_RGx*L_RGx+L_RGz*L_RGz))
// #define L_JBW   (sqrt(L_JBWx*L_JBWx+L_JBWz*L_JBWz))

#endif // _LINKCONF_VER2_H_