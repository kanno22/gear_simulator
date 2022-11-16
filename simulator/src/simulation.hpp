#ifndef _SIMULATION_CPP_
#define _SIMULATION_CPP_

#include <fstream>
#include <mutex>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include"model.hpp"

// #define MODE_FLOAT 0
// #define MODE_STANCE 1
// #define TORQUE_PARTITON 4

using namespace std;
using namespace Eigen;
using std::vector;

class Simulation
{
    private:

    bool end_flag;
    stateClass preState;
    double fric_force[3];
    double w_fric_force[3];
    double torque[3];
    double target_jointAngle=0;
    void update_input();
    vector<vector<double>> input_series;
    void read_torque();
    vector<string> read_data(string& input, char delimiter);
    void control_jointAngle(double degree);
    void control_motorAngle(double degree);
    void excite(double torque);
    void fb_excite(double torque);
    void fb_excite_bl(double torque);

    public:
    Simulation();
    stateClass currentState;//現在のロボットの状態変数
    double timer;
    // double reward;
    // double penalty;
    // double max_penalty;
    // double penalty_back;
    // double penalty_link;
    // double max_torque;
    // double max_link;
    // double min_link;
    double control_target;
    // bool done;
    // int pause;
    modelClass model;
    static std::ofstream log;
    static std::ofstream log_opti;
    std::once_flag log_once;
    stateClass reset_simulation();
    stateClass calc_reactForce(stateClass currentState);
    stateClass sim_calc();
    
    void simu_loop(stateClass& state);
    double get_ground(double x);
    void logging();
    void log_init();
   
    // vector<double> get_state();


};

#endif
