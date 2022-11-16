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
// #define DEBUG_REACT_FORCE
// #define CHECK_SIMU  
// #define PRINT_LOADED_DATA

#define T_UP 1
double finish_time = 100;

using namespace Eigen;
using std::cout;
using std::endl;
using std::ios;
using Eigen::MatrixXd ;
using Eigen::Vector2d;

typedef Eigen::Matrix<double, 5, 1> Vector5d;
typedef Eigen::Matrix<double, 5, 5> Matrix5d;    

extern Timer system_time;

double delta_t = 1E-3;  

std::ofstream Simulation::log;
std::ofstream Simulation::log_opti;

Simulation::Simulation(){
  timer = 0;  
  end_flag = 0;
  reset_simulation();
  read_torque();
}

stateClass Simulation::reset_simulation(){
  timer = 0;  
  end_flag = 0; 
  // done = 0;
  // max_penalty=0;
  // penalty_back = 0;
  // penalty_link = 0;
  // max_torque = 0;
  // max_link =0;
  // min_link = 0;
  currentState.reset_state();
  
  log_init();
  return currentState;
}


vector<string> Simulation::read_data(string& input, char delimiter)//いらない
{
    istringstream stream(input);//string型をstringstream型にする（,をいれるため）
    string field;
    vector<string> result;
    while (getline(stream, field, delimiter)) {//stringstream型の文字列を,で分割しfieldに一つずつ格納
        result.push_back(field);
    }
    return result;
}

void Simulation::read_torque()//いらない
{
    ifstream ifs("input.csv");
    string line;
    getline(ifs, line);//lineに格納
    while (getline(ifs, line)) 
    {
      vector<string> strvec = read_data(line, ',');//line内の文字列を,で分割してstrvecに一つずつ格納
      vector<double> moment(0,0);//0個0を入れる？意味なくね？
      for (int i=0; i<strvec.size();i++)
      {
        moment.push_back(stod(strvec.at(i)));//string型をdouble型に変換してmomentに格納
      }
      input_series.push_back(moment);
    }

    std::cout << "FINISH LOADING DATA" <<std::endl;
    
}


void Simulation::log_init(){
  log.open("simulation_log.csv",ios::trunc);
  log << "time\t" 
      << "x\t" << "z\t" << "Theta_r\t" << "Theta_j\t"
      << "dx/dt\t" << "dz/dt\t" << "d(Theta_r)/dt\t" << "d(Theta_j)/dt\t"
      << "ff_f\t" << "ff_m\t" <<"ff_b\t"
      << "wf_f\t" << "wf_m\t" <<"wf_b\t"
      << "torque_f\t" << "torque_m\t" <<"torque_b\t"
      << "spring torque"
      // ;for(int i=0;i<11;i++) log << i << "\t" 
      << endl;
  logging();
}

void Simulation::logging(){
  log << timer << "\t" 
                  ;for(int i=0;i<4;i++) log << currentState.pose[i] << "\t"; log   
                  ;for(int i=0;i<4;i++) log << currentState.velo[i] << "\t"; log
                  << fric_force[0] <<"\t" <<fric_force[1] << "\t" << fric_force[2] << "\t"
                  << w_fric_force[0] <<"\t" << w_fric_force[1] << "\t" << w_fric_force[2] << "\t"
                  ;for(int i=0;i<3;i++) log << currentState.torque[i] << "\t"; log
                  << currentState.springTorque << "\t"
                  << target_jointAngle *M_PI/180. << "\t"
                  << endl;
}


void Simulation::update_input()
{//currentState.external_forces[4]（モータリンク）に値を代入すればよい
//その他はいらない
  static int count=-1;
  if(count+1<input_series.size()-1)
  {
    if(timer>input_series[count+1][0])//現在時刻よりcount+1行0列にある時刻が小さい時（0.1>0.005）
    {
      std::cout<<"---UPDATE INPUT---"<<std::endl;
      count++;
      for(const auto e: input_series[count])
      {
        std::cout<< e <<" ";
      }
      std::cout<<count<<" ";
      std::cout<<std::endl;
      switch ((int)input_series[count][1])
      {
        case 0: std::cout<<"Torque control"<<std::endl; break;
        case 1: std::cout<<"Angle control"<<std::endl; break;
        case 3: std::cout<<"SEA: Torque control"<<std::endl; break;
        case 4: std::cout<<"SEA: Angle control"<<std::endl; break;
        default: std::cout<<"ERROR: value of mode "<<std::endl; break;
      }
    }
  }
  if(timer>input_series[0][0])
  {
    currentState.torque[0] = input_series[count][2] / WHEEL_R;//後輪
    currentState.torque[1] = input_series[count][3] / WHEEL_R;//中輪
    currentState.torque[2] = input_series[count][4] / WHEEL_R;//前輪
    switch ((int)input_series[count][1])
    {
      case 1:
      target_jointAngle = input_series[count][5];
      control_jointAngle(target_jointAngle);
      break;
      
      case 0:
      currentState.external_forces[3] += input_series[count][5];
      break;

      case 3:
      currentState.external_forces[4] = input_series[count][5];
      break;

      case 4:
      control_target = input_series[count][5];
      control_motorAngle(input_series[count][5]);
      break;

      case 5:
      std::cout << "excite mode" << std::endl;
      excite(input_series[count][5]);
      break;

      case 6:
      // std::cout<<"hoge"<<std::endl;
      std::cout << "FB_excite mode" << std::endl;
      fb_excite(input_series[count][5]);
      break;

      case 7:
      std::cout<<"input error " << (int)input_series[count][1]<<std::endl;
      fb_excite_bl(input_series[count][5]);
      break;

      default:


      break;
    }
  }
}

void Simulation::control_jointAngle(double degree){
  double jointTorque = 0.004 * (degree*M_PI/180. - currentState.pose[3]) - 0.001*currentState.velo[3];
  if(jointTorque>MAX_JOINT_TORQUE) jointTorque=MAX_JOINT_TORQUE;
  if(jointTorque<-MAX_JOINT_TORQUE) jointTorque=-MAX_JOINT_TORQUE;
  currentState.external_forces[4] += jointTorque;
  // double jointTorque = 0.002 * (degree*M_PI/180. - currentState.pose[4]) - 0.0008*currentState.velo[4];
  // if(jointTorque>MAX_JOINT_TORQUE) jointTorque=MAX_JOINT_TORQUE;
  // if(jointTorque<-MAX_JOINT_TORQUE) jointTorque=-MAX_JOINT_TORQUE;
  // currentState.external_forces[4] += jointTorque;
  // double jointTorque = P_CONTROL_JOINT * (degree*M_PI/180. - currentState.pose[3]);
  // if(jointTorque>MAX_JOINT_TORQUE) jointTorque=MAX_JOINT_TORQUE;
  // if(jointTorque<-MAX_JOINT_TORQUE) jointTorque=-MAX_JOINT_TORQUE;
  // currentState.external_forces[4] += jointTorque;
}

void Simulation::control_motorAngle(double degree){
  double jointTorque = 10 * (degree*M_PI/180. - currentState.pose[4]) - 0.3*currentState.velo[4];
  if(jointTorque>MAX_J_MOTOR_TORQUE) jointTorque = MAX_J_MOTOR_TORQUE;
  if(jointTorque<-MAX_J_MOTOR_TORQUE) jointTorque = -MAX_J_MOTOR_TORQUE;
  currentState.external_forces[4] = jointTorque;
  std::cout << "DEBUG : control_motorAngle " << std::endl;
  std::cout << "jointTorque: " << jointTorque << "  deviation: " << (degree*M_PI/180. - currentState.pose[4]) << " D_element: " << -0.001*currentState.velo[4] <<  std::endl ;
}

void Simulation::excite(double time){
  double d_t = time;
  static double change_time = timer + d_t;
  double torque = currentState.external_forces[4];
  // std::cout << "time " << timer 
  if(timer> change_time){
    torque *= -1;
    change_time = timer + d_t;
  } 
  currentState.external_forces[4] = torque;
}

void Simulation::fb_excite(double torque){
  if(currentState.velo[4]>0) currentState.external_forces[4] = torque;
  else if(currentState.velo[4]<0) currentState.external_forces[4] = -torque;
  else currentState.external_forces[4] = 0;
  // std::cout<<"fuga"<<std::endl;
}

void Simulation::fb_excite_bl(double torque){
  if(currentState.velo[3]>0) currentState.external_forces[4] = torque;
  else if(currentState.velo[3]<0) currentState.external_forces[4] = -torque+0.4;
  else currentState.external_forces[4] = 0;
  std::cout<<"error"<<std::endl;
}

stateClass Simulation::sim_calc(){
  preState = currentState; 
  currentState.kinematics();//現在の各リンクの位置を計算
  
  currentState = calc_reactForce(currentState);//現在の床反力を計算
  update_input();//入力

  currentState = RungeKutta(currentState, modelClass::eq_differential);//運動方程式を解く
  timer += delta_t;
  
  return currentState;
}

void Simulation::simu_loop(stateClass& state){//メインループ
  state = reset_simulation();//パラメータをリセット
  std::ofstream _log;
  _log.open("simulation_log.csv",ios::trunc);
  _log << "time[sec]\t" 
      << "x[m]\t" << "z[m]\t" << "Theta_r[deg]\t" << "Theta_j[deg]\t"
      << "dx/dt[m/s]\t" << "dz/dt[m/s]\t" << "d(Theta_r)/dt[rad/sec]\t" << "d(Theta_j)/dt[rad/sec]\t"
      << "torque_forward[Nm]\t" << "torque_middle[Nm]\t" <<"torque_back[Nm]\t"<<"torque_joint[Nm]\t"
      << "Control Target\t" 
      << "angle between rocker bogie & motor link\t"
      << "motor torque\t"
      << "theta_m"
       
      << endl;
  std::getchar();
  double time_offset = system_time.get_current_milli();
  while (!end_flag)//繰り返し
  { 
    if(1){
      state = sim_calc();
      _log  << timer << "\t"
            << currentState.pose[0] << "\t"
            << currentState.pose[1] << "\t"
            << currentState.pose[2]*180/M_PI << "\t"
            << currentState.pose[3]*180/M_PI << "\t"   
            ;for(int i=0;i<4;i++) _log << currentState.velo[i] << "\t"; _log      
            ;for(int i=0;i<3;i++) _log << currentState.torque[i]*WHEEL_R << "\t"; _log
            <<currentState.external_forces[3] << "\t"
            << control_target << "\t"
            << (currentState.pose[4] - currentState.pose[3])* 180./M_PI<< "\t"
            << currentState.external_forces[4] << "\t"
            << (currentState.pose[4]) *180./M_PI << "\t"
            << endl;
      #ifdef STOP_SIMULATION
        while( std::getchar() != '\n');
      #endif 
    }
    if (timer > finish_time) end_flag = 1;
    else end_flag =0;
  }
}

stateClass Simulation::calc_reactForce(stateClass currentState)
{
  double &f_x = currentState.joint[4].x;//前輪位置
  double &m_x = currentState.joint[3].x;//中輪位置
  double &b_x = currentState.joint[2].x;//後輪位置
  double &f_z = currentState.joint[4].z;  
  double &m_z = currentState.joint[3].z;
  double &b_z = currentState.joint[2].z;

  for(int i=0; i<4; i++)
  {
    currentState.external_forces[i]=0;//外力をリセット
  }
  currentState.get_wheel_h_min();//現在の車輪高さを取得

  if(1)
  {
    double reactForce;
    double wall_reactfroce;
    
    double torque_wall=0;
   
   //frontwheel
    if((f_z-WHEEL_R)<get_ground(f_x) || ( (f_x+WHEEL_R)>STEP_X && f_z<STEP_HIGHT) || ((pow(f_z-STEP_HIGHT,2)+pow(f_x-STEP_X,2))< pow(WHEEL_R,2)))
    {//前輪高さが地面にめり込んでいるor壁にめり込んでいる
      double torque_z=currentState.torque[1];
      double hoge=abs(currentState.torque[1]);

      if((f_z-WHEEL_R)<get_ground(f_x)) reactForce = (-K_WHEEL*(f_z-WHEEL_R-get_ground(f_x)) + D_WHEEL*currentState.wheel_velo_f);//前輪が地面にめり込んでいるとき床反力=-kz+cdz
      else reactForce =0;//めり込んでいないとき床反力=0

      if((f_x+WHEEL_R)>STEP_X && f_z<STEP_HIGHT)//壁にめり込んでいるとき(いらない)
      {
        wall_reactfroce = (-K_WHEEL*((f_x+WHEEL_R)-STEP_X) + D_WHEEL*currentState.wheel_velo_f_x);//壁からの反力=-kx+cdx
        torque_wall = currentState.torque[0];
      }
      else if(f_x<STEP_X && f_z>STEP_HIGHT &&((pow(f_z-STEP_HIGHT,2) + pow(f_x-STEP_X,2))< pow(WHEEL_R,2) ) )
      {
        torque_wall = currentState.torque[0];
        double th_rf = atan2(STEP_HIGHT-f_z,STEP_X-f_x);
        wall_reactfroce = (-K_WHEEL*((f_x+WHEEL_R*cos(th_rf)-STEP_X)) + D_WHEEL*currentState.wheel_velo_f_x);
        reactForce =  (-K_WHEEL*(f_z+WHEEL_R*sin(th_rf)-STEP_HIGHT) + D_WHEEL*currentState.wheel_velo_f);
      }
      else wall_reactfroce =0;

      if( abs(torque_z) > abs(wall_reactfroce) )
      {
        if(torque_z<0) torque_z= -wall_reactfroce;
        else torque_z = wall_reactfroce;
      }

      if( abs(hoge) > abs(wall_reactfroce) )
      {
        hoge = abs(wall_reactfroce);
      } 
      w_fric_force[2] = abs(hoge);  
    
      double theta_fw = -atan( (currentState.joint[4].x - currentState.joint[1].x)/(currentState.joint[4].z - currentState.joint[1].z) );  
      double theta_j = -atan( (currentState.joint[1].x - currentState.joint[0].x)/(currentState.joint[1].z - currentState.joint[0].z) );  
      double theta_jw = atan2( (currentState.joint[4].z - currentState.joint[1].z),(currentState.joint[4].x - currentState.joint[1].x) );
      double theta_rj = atan2( (currentState.joint[1].z - currentState.joint[0].z),(currentState.joint[1].x - currentState.joint[0].x) ); 
      double torque = currentState.torque[0];

      if(reactForce<0) reactForce=0;
      if(abs(torque)>abs(reactForce)){
        if(torque<0) torque= -reactForce;
        else torque = reactForce;
      }

      // torque *= 2;
      torque_wall = 0;
      double f_x = torque + wall_reactfroce;//前輪による水平力+壁からの反力
      double f_z = reactForce + torque_wall;//床反力+壁からの水平力
      double f_r = (reactForce + torque_wall)*(currentState.joint[1].x-currentState.joint[0].x)
                  -(torque + wall_reactfroce)*(currentState.joint[1].z-currentState.joint[0].z);
      double f_j = (reactForce + torque_wall)*(currentState.joint[4].x-currentState.joint[1].x)
                  -(torque*T_UP + wall_reactfroce)*(currentState.joint[4].z-currentState.joint[1].z);
      currentState.external_forces[0] += f_x;
      currentState.external_forces[1] += f_z;
      currentState.external_forces[2] += f_r;
      currentState.external_forces[3] += f_j;

    }

    // middle wheel
    if((m_z-WHEEL_R)<get_ground(m_x) || ( (m_x+WHEEL_R)>STEP_X && m_z<STEP_HIGHT) || (( pow(m_z-STEP_HIGHT,2) + pow(m_x-STEP_X,2))< pow(WHEEL_R,2) ) ){
      double torque_z=currentState.torque[1];
      double hoge=abs(currentState.torque[1]);

      if((m_z-WHEEL_R)<get_ground(m_x)) reactForce = (-K_WHEEL*(m_z-WHEEL_R-get_ground(m_x)) + D_WHEEL*currentState.wheel_velo_m);
      else reactForce =0;
      if(reactForce<0)reactForce=0;
      
      if((m_x+WHEEL_R)>STEP_X && m_z<STEP_HIGHT){
        torque_wall = currentState.torque[1];
        wall_reactfroce = (-K_WHEEL*((m_x+WHEEL_R)-STEP_X) + D_WHEEL*currentState.wheel_velo_m_x);
      }
      else if(m_x<STEP_X && m_z>STEP_HIGHT && (pow(m_z-STEP_HIGHT,2) + pow(m_x-STEP_X,2))<pow(WHEEL_R,2)){
        // cout<<"\n\nm_kado\n\n";
        torque_wall = currentState.torque[1];
        double th_rf = atan2(STEP_HIGHT-m_z,STEP_X-m_x);
        wall_reactfroce = (-K_WHEEL*((m_x+WHEEL_R*cos(th_rf)-STEP_X)) + D_WHEEL*currentState.wheel_velo_m_x);
        reactForce =  (-K_WHEEL*(m_z+WHEEL_R*sin(th_rf)-STEP_HIGHT) + D_WHEEL*currentState.wheel_velo_m);
        #ifdef DEBUG_REACT_FORCE
        cout << "\n\t" << (m_x+WHEEL_R*cos(th_rf)-STEP_X) <<"\t" << m_x <<"\t" << WHEEL_R*cos(th_rf) <<"\t" <<STEP_X;
        cout << "\n\t" << th_rf  << "\t" << wall_reactfroce << "\t" <<  reactForce;
        #endif
      }
      else wall_reactfroce =0;
      
      if( abs(torque_z) > abs(wall_reactfroce) ){        
        if(torque_z<0) torque_z= -wall_reactfroce;
        else torque_z = wall_reactfroce;
      }
      if( abs(hoge) > abs(wall_reactfroce) )
      {
        hoge = abs(wall_reactfroce);
      }  
      w_fric_force[1] = abs(hoge);  
      
      double theta_mw = -atan( (currentState.joint[3].x - currentState.joint[1].x)/(currentState.joint[3].z - currentState.joint[1].z) );  
      double theta_j = atan( (currentState.joint[1].x - currentState.joint[0].x)/(currentState.joint[1].z - currentState.joint[0].z) );  
      double theta_j_mw = (theta_j- theta_mw);
      double theta_jw = atan2( (currentState.joint[3].z - currentState.joint[1].z),(currentState.joint[3].x - currentState.joint[1].x) );
      double theta_rj = atan2( (currentState.joint[1].z - currentState.joint[0].z),(currentState.joint[1].x - currentState.joint[0].x) ); 
      double torque = currentState.torque[1];

      if(abs(torque)>abs(reactForce)){
        if(torque<0) torque= -reactForce;
        else torque = reactForce;
      } 
      fric_force[1] = torque;
  
      
      // torque *= 2;
      torque_wall = 0;
      double f_x = torque + wall_reactfroce;
      double f_z = reactForce + torque_wall;
      double f_r = (reactForce + torque_wall)*(currentState.joint[1].x-currentState.joint[0].x)
                  -(torque + wall_reactfroce)*(currentState.joint[1].z-currentState.joint[0].z);
      double f_j = (reactForce + torque_wall)*(currentState.joint[3].x-currentState.joint[1].x)
                  -(torque*T_UP+wall_reactfroce)*(currentState.joint[3].z-currentState.joint[1].z);
      currentState.external_forces[0] += f_x;
      currentState.external_forces[1] += f_z;
      currentState.external_forces[2] += f_r;
      currentState.external_forces[3] += f_j;


 
      reactForce = 0;
    }
    else fric_force[1]=0;

    // === back wheel ===
    if((b_z-WHEEL_R)<get_ground(b_x) || ( (b_x+WHEEL_R)>STEP_X && b_z<STEP_HIGHT) || (( pow(b_z-STEP_HIGHT,2) + pow(b_x-STEP_X,2))< pow(WHEEL_R,2) ) ){
      double torque_z=currentState.torque[2];
      double hoge=abs(currentState.torque[2]);
      
      if((b_z-WHEEL_R)<get_ground(b_x)) reactForce = (-K_WHEEL*(b_z-WHEEL_R-get_ground(b_x)) + D_WHEEL*currentState.wheel_velo_b);
      else reactForce =0;

      if((b_x+WHEEL_R)>STEP_X && b_z<STEP_HIGHT){
        wall_reactfroce = (-K_WHEEL*((b_x+WHEEL_R)-STEP_X) + D_WHEEL*currentState.wheel_velo_b_x);
        torque_wall = currentState.torque[2];
      }
      else if(b_x<STEP_X && b_z>STEP_HIGHT &&(pow(b_z-STEP_HIGHT,2) + pow(b_x-STEP_X,2))< pow(WHEEL_R,2) ){
        // cout<<"\n\nm_kado\n\n";
        torque_wall = currentState.torque[2];
        double th_rf = atan2(STEP_HIGHT-b_z,STEP_X-b_x);
        wall_reactfroce = (-K_WHEEL*((b_x+WHEEL_R*cos(th_rf)-STEP_X)) + D_WHEEL*currentState.wheel_velo_b_x);
        reactForce =  (-K_WHEEL*(b_z+WHEEL_R*sin(th_rf)-STEP_HIGHT) + 0*D_WHEEL*currentState.wheel_velo_b);
        #ifdef DEBUG_REACT_FORCE
        cout << "\n\t" << (b_x+WHEEL_R*cos(th_rf)-STEP_X) <<"\t" << b_x <<"\t" << WHEEL_R*cos(th_rf) <<"\t" <<STEP_X;
        cout << "\n\t" << th_rf  << "\t" << wall_reactfroce << "\t" <<  reactForce;
        #endif
      }
      else wall_reactfroce =0;
      
      if( abs(torque_z) > abs(wall_reactfroce) ){
        if(torque_z<0) torque_z= -wall_reactfroce;
        else torque_z = wall_reactfroce;
      }
      if( abs(hoge) > abs(wall_reactfroce) ){
        hoge = abs(wall_reactfroce);
      } 
      w_fric_force[2] = abs(hoge);  
  
      double _theta = atan( (currentState.joint[2].x - currentState.joint[0].x)/(currentState.joint[2].z - currentState.joint[0].z) );  
      double theta_m = atan( (currentState.joint[0].z - currentState.joint[2].z)/(currentState.joint[0].x - currentState.joint[2].x) );  
      double theta_rw = atan( (currentState.joint[2].z - currentState.joint[0].z)/(currentState.joint[2].x - currentState.joint[0].x) );
      double torque = currentState.torque[2];
      if(abs(torque)>abs(reactForce)){
        if(torque<0) torque= -reactForce;
        else torque = reactForce;
      }
      fric_force[2] = currentState.torque[2];
      if(abs(torque)>abs(reactForce)*2){
        if(torque<0) torque= -reactForce;
        else torque = reactForce;
      }
      
      // torque *= 2;
      torque_wall = 0;
      double f_x = torque + wall_reactfroce;
      double f_z = reactForce + torque_wall;
      double f_r = (reactForce + torque_wall)*(currentState.joint[2].x-currentState.joint[0].x)
                  -(torque + wall_reactfroce)*(currentState.joint[2].z-currentState.joint[0].z);
      // double f_j = -(reactForce + torque_wall)*(currentState.joint[2].x-currentState.joint[1].x);
      //             +(torque + wall_reactfroce)*(currentState.joint[2].z-currentState.joint[1].z);
      double f_j = 0;
                  // +(torque + wall_reactfroce)*(currentState.joint[2].z-currentState.joint[1].z);
      currentState.external_forces[0] += f_x;
      currentState.external_forces[1] += f_z;
      currentState.external_forces[2] += f_r;
      currentState.external_forces[3] += f_j;
      

      reactForce = 0;
    }
    else fric_force[2]=0;
  }

  for(int i=0;i<3;i++){
    if(fric_force[2]<0) fric_force[2]=0;
  }
  return currentState;
} 

// vector<double> Simulation::get_state(){
//   vector<double> state(10,0);
//   for(int i=0;i<5;i++){
//     state[i] = currentState.pose[i];
//     state[i+5] = currentState.velo[i];
//   }
//   return state;
// }

double Simulation::get_ground(double x)
{
  if(x < STEP_X)
  {
    return 0;
  }
  else return STEP_HIGHT; 
}