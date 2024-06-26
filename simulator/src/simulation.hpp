#ifndef _SIMULATION_CPP_
#define _SIMULATION_CPP_

#include <fstream>
#include <mutex>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include"model.hpp"

using namespace std;
using namespace Eigen;
using std::vector;

class Simulation
{
    private:

    bool end_flag;
    // double torque[3];
    //PD制御用の変数 
    Matrix<double, 6, 1> pose_ref;
    Matrix<double, 6, 1> error;
    Matrix<double, 6, 1> olderror;

    //状態フィードバック用の変数
    Matrix<double,4,1> state;
    Matrix<double,4,1> oldstate;
    Matrix<double,4,1> state_ref;
    Matrix<double,1,4> K;//状態フィードバックゲイン
    
    //角度励振用のパラメータ
    double la;
    double lmax;
    double fg;
    double wg;
    double fap;
    double wap;

    double tto;
    double tg;
    int ttocount;
    int ttocounter;

    //極値
    double oldbane[2];
    int n;
    int ncount;

    void update_input();
    int extremum();//極値の判定 n回目の極少値1 それ以外0

    public:
    Simulation();
    stateClass currentState;//現在のロボットの物理量
    double timer;
    modelClass model;
    static std::ofstream log;
    static std::ofstream log_opti;
    std::once_flag log_once;
    stateClass reset_simulation();
    stateClass calc_reactForce(stateClass currentState);
    stateClass sim_calc();
    
    void PD();//角度制御
    void AngleExcitation();//角度制御で励振
    void AngleExcitation_2();//空中区間も含む
    
    void AngleExcitation_f();//最終版
    void BodyAngle();//ボディリンク目標角度ジェネレータ
    void fbExcitation();//フィードバック励振


    void Statefeedback();//状態フィードバック
    void StateGenerator();//目標状態変数を生成

    void simu_loop(stateClass& state);
    double get_ground(double x);
    void logging();
    void log_init();

};

#endif
