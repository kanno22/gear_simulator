#ifndef _LINKCONF_VER2_H_
#define _LINKCONF_VER2_H_
#include <cmath>


// -----変える必要有り ここから↓-----

//ロッカーリンクの質量[kg]
#define M_R      1.78
//ボギーリンクリンクの質量[kg]
#define M_B      1.01

//リンク長さ[m]
#define L_BGx      (0.0)
#define L_BGz      (0.03)
#define L_RGx      (0.05)
#define L_RGz      (0.05)
#define L_RGWx  (0.1)
#define L_RGWz (0.1)
#define L_JBWx  (0.05)
#define L_JBWz  (0.05)
// バネなしの場合は変更不要---ここから
#define L_C    (0.02)
#define L_RJ     (0.065)
#define L_M     (0.02)
#define L_KN     0.038
// バネなしの場合は変更不要---ここまで

// ロッカーリンクの重心まわりの慣性モーメント[kg m^2]
#define	J_R		2*16537791E-9
// ボギーリンクの重心まわりの慣性モーメント[kg m^2]
#define	J_B		2*11307607E-9
#define J_M     2855872E-9

// 車輪半径[m]
#define WHEEL_R (0.025)

// -----変える必要有り ここまで↑-----

// -----必要に応じて変更 ここから↓-----
// // ロボットの初期位置 
// // x座標[m]
// #define X_START -0.125
// // z座標[m]
// #define Z_START (WHEEL_R+L_RGWx)
// // ロッカーリンクの角度[deg]
// #define THETA_Rocker_START 0
// // 関節角度[deg]
// #define THETA_Joint_START 0

// 段差の水平位置[m]
#define STEP_X 0//(0.03-0.03)
// 段差の高さ[m]
#define STEP_HIGHT (0.05)
// #define STEP_HIGHT (0.0)

// 最大車軸トルク[N m]
#define MAX_TORQUE 5
// 最大関節トルク[N m]
// #define MAX_JOINT_TORQUE 2.5
#define MAX_JOINT_TORQUE 10
// 最大関節モータトルク[N m]
#define  MAX_J_MOTOR_TORQUE 10

// ばね係数（バネなしの場合は0）
// #define K_SPRING   0.2
#define K_SPRING   3000

// ばねのオフセット角度
#define THETA_JO (0*M_PI/180)

// リンク関節の摩擦係数
#define C_JOINT_FRICTION (0.001)
#define C_MOTOR_FRICTION (0.02)
// #define C_MOTOR_FRICTION (0.000)

// / 関節角度制御のPゲイン
#define P_CONTROL_JOINT 70

// 重力加速度
#define G_ACCEL 9.8

// ホイールの粘弾性の設定
#define K_WHEEL 5000
#define D_WHEEL -100

// シミュレーションの描画の倍速
// #define TIME_SLOW_RATE 0.25

// ディスプレイ設定
// 描画の大きさ
#define IMAGE_WIDTH 720.0
#define IMAGE_HEIGHT 480.0
#define RATIO_XY (IMAGE_WIDTH /IMAGE_HEIGHT) 
// ロボットの描画位置
#define Y_DISP -0.5
#define X_DISP 0
// ロボットの描画の大きさ
#define PLOT_RATE 4

#define X_WINDOWSIZE 600.0
#define Y_WINDOWSIZE 900.0

// -----必要に応じて変更 ここまで↑-----

// 変更不要
#define L_RGW   (sqrt(L_RGWx*L_RGWx+L_RGWz*L_RGWz))
#define L_RGJ   (sqrt(L_RGx*L_RGx+L_RGz*L_RGz))
#define L_JBW   (sqrt(L_JBWx*L_JBWx+L_JBWz*L_JBWz))

#endif // _LINKCONF_VER2_H_