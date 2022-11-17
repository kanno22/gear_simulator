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
    // //PID制御用の変数 
    // /Eigen::Matrix<double, 6, 1> pose
    // // double error;
    // // double olderror;
    // //
    void update_input();

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
    
    //stateClass AngleControl(stateClass currentState);

    void simu_loop(stateClass& state);
    double get_ground(double x);
    void logging();
    void log_init();

};

#endif
