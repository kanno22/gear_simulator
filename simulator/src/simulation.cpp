#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "simulation.hpp"
#include "linkconf_ver2.hpp"
#include "model.hpp"
#include "RungeKutta.hpp"
#include "timer.hpp"

#define TIME_SLOW_RATE 1

#define STOP_SIMULATION

#define T_UP 1
double finish_time = 100;

using namespace Eigen;
using std::cout;
using std::endl;
using std::ios;
using Eigen::MatrixXd ;
using Eigen::Vector2d;

// typedef Eigen::Matrix<double, 5, 1> Vector5d;
// typedef Eigen::Matrix<double, 5, 5> Matrix5d;    

extern Timer system_time;

double delta_t = 1E-3;  

std::ofstream Simulation::log;
std::ofstream Simulation::log_opti;

Simulation::Simulation()
{
  timer = 0;  
  end_flag = 0;
  reset_simulation();
}

stateClass Simulation::reset_simulation()
{
  timer = 0;  
  end_flag = 0; 
  currentState.reset_state();
  
  log_init();
  return currentState;
}

void Simulation::log_init()
{
  log.open("simulation_log.csv",ios::trunc);
  log << "time\t" 
      << "x\t" << "z\t" << "Theta_1\t" << "Theta_m\t"<<"Theta_2\t"<<"Theta_3\t"
      <<"wheel torque\t"
      << "motor torque\t"
      << "body torque\t"
      << endl;
  logging();
}

void Simulation::logging()
{
  log  << timer << "\t"
       << currentState.pose[0] << "\t"
       << currentState.pose[1] << "\t"
       << currentState.pose[2]*180/M_PI << "\t"
       << currentState.pose[3]*180/M_PI << "\t"
       << currentState.pose[4]*180/M_PI << "\t"
       <<currentState.pose[5]*180/M_PI << "\t"
       << currentState.torque << "\t"
       <<currentState.external_forces[3] << "\t"
       << currentState.external_forces[4] << "\t"
       << endl;
}

void Simulation::update_input()
{
 currentState.external_forces[4]=0.0;//大腿リンクへの入力
 currentState.external_forces[5]=0.0;//ボディリンクへの入力

}

stateClass Simulation::sim_calc()
{
  currentState.kinematics();//順運動学計算
  
  currentState = calc_reactForce(currentState);//床反力を算出
  update_input();//入力

  currentState = RungeKutta(currentState, modelClass::eq_differential);//運動方程式を解く
  timer += delta_t;
  
  return currentState;
}

void Simulation::simu_loop(stateClass& state)//メイン
{
  state = reset_simulation();//パラメータをリセット
  std::ofstream _log;
  _log.open("simulation_log.csv",ios::trunc);
  _log << "time[sec]\t" 
      << "x[m]\t" << "z[m]\t" << "Theta_r[deg]\t" << "Theta_j[deg]\t"
      << "dx/dt[m/s]\t" << "dz/dt[m/s]\t" << "d(Theta_r)/dt[rad/sec]\t" << "d(Theta_j)/dt[rad/sec]\t"
      <<"wheel torque\t"
      << "motor torque\t"
      << "body torque\t"
      << endl;
  std::getchar();
  double time_offset = system_time.get_current_milli();

  while (!end_flag)//メインループ
  { 
      state = sim_calc();
      _log  << timer << "\t"
            << currentState.pose[0] << "\t"
            << currentState.pose[1] << "\t"
            << currentState.pose[2]*180/M_PI << "\t"
            << currentState.pose[3]*180/M_PI << "\t"
            << currentState.pose[4]*180/M_PI << "\t"
            <<currentState.pose[5]*180/M_PI << "\t"
            << currentState.torque*WHEEL_R << "\t"
            <<currentState.external_forces[3] << "\t"
            << currentState.external_forces[4] << "\t"
            << endl;
       #ifdef STOP_SIMULATION
        while( std::getchar() != '\n');
      #endif 
    
    if (timer > finish_time) end_flag = 1;
    else end_flag =0;
  }
}

stateClass Simulation::calc_reactForce(stateClass currentState)
{
  double f_x= currentState.joint[0].x;
  double f_z = currentState.joint[0].z;  

  for(int i=0; i<4; i++)
  {
    currentState.external_forces[i]=0;//外力をリセット
  }
  currentState.get_wheel_h_min();//現在の車輪高さを取得

  double reactForce;

    if((f_z-WHEEL_R)<get_ground(f_x) )
    {//前輪高さが地面にめり込んでいるor壁にめり込んでいる
      double torque_z=currentState.torque;
      double hoge=abs(currentState.torque);

      if((f_z-WHEEL_R)<get_ground(f_x)) 
      {
        if(currentState.velo[1]<0)
        {
          reactForce = -K_WHEEL*(f_z-WHEEL_R-get_ground(f_x)) - C_WHEEL*currentState.velo[1];
        }
        else 
        {
          reactForce = -K_WHEEL*(f_z-WHEEL_R-get_ground(f_x));
        }
        
      }
      else reactForce =0;

      if(reactForce<0) reactForce=0;

      double torque =currentState.torque;
      double f_x = torque;//前輪による水平力
      double f_z = reactForce;//床反力

      currentState.external_forces[0] += f_x;
      currentState.external_forces[1] += f_z;
    }
  return currentState;
}

double Simulation::get_ground(double x)
{
  if(x < STEP_X)
  {
    return 0;
  }
  else return STEP_HIGHT; 
}