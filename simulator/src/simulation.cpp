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

//#define STOP_SIMULATION
//#define STOP_TIMER
#define STOP_TIME 6//8//0

#define AExcitation
//#define Excitation
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
//
  pose_ref<<0,0,0,0,0,0;
  error<<0,0,0,0,0,0;
  olderror<<0,0,0,0,0,0;

//
  la=21.5625*(M_PI/180);//20
  lmax=2*THETA_2_START*(M_PI/180);
  fg=1.5;//1.5;//3.817;//2
  wg=-2*M_PI*fg;
  fap=2.7;
  wap=-2*M_PI*fap;

  tto=0;
  ttocount=-1;//床反力が最初0Nだから
  ttocounter=0;
  tg=0;
//
  state<<0,0,0,0;
  oldstate<<0,0,0,0;
  state_ref<<X_START,0,0,0;
  //K<<-0.208058,-0.416116,-14.9185,-1.18899;//-2,-2,-2,-2
  K<<0,-0.416116,-14.9185,-1.18899;//位置指令なし


  oldbane[0]=0;
  oldbane[1]=0;
  n=0;//２回目の振動の極大値
  ncount=0;

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
      <<"reaction force\t"
      <<"wheel torque\t"//WHEEL MOTOR TORQUE
      << "angle2 torque\t"//BODYLINK JOINT TORQUE
      << "motor torque\t"//BODYLINK JOINT TORQUE
      << "body torque\t"//MOTORLINK JOINT TORQUE
      << "spring torque\t"
      <<"knee angle\t"
      << "Theta_2_ref\t"
      <<"CoD_z\t"
      <<"Wheel_speed\t"
      <<"Wheel_speed_ref\t"
      <<"Wheel_anglespeed\t"
      <<"Wheel_rpm\t"
      <<"Wheel_output\t"
      << endl;
  logging();
}

void Simulation::logging()
{
  log  << timer << "\t"
       <<currentState.pose[0] << "\t"
       <<currentState.pose[1] << "\t"
       <<currentState.pose[2]*180/M_PI << "\t"
       <<currentState.pose[3]*180/M_PI << "\t"
       <<currentState.pose[4]*180/M_PI << "\t"
       <<currentState.pose[5]*180/M_PI << "\t"
       <<currentState.external_forces[1] << "\t"
       <<currentState.torque << "\t"
       <<currentState.external_forces[4] << "\t"
       <<-1*currentState.external_forces[4] << "\t"//モータリンクから見た
       << currentState.external_forces[5] << "\t"
       <<-1*currentState.pose[3]*K_SPRING<< "\t"
       <<(currentState.pose[3]+currentState.pose[4])*180/M_PI << "\t"
       <<pose_ref[4]*180/M_PI <<"\t"
       <<currentState.joint[7].z<<"\t"
       <<currentState.wheel_velo<<"\t"
       <<currentState.wheel_velo_ref<<"\t"
       <<currentState.wheel_velo/WHEEL_R<<"\t"
       <<(currentState.wheel_velo/WHEEL_R)*(30/M_PI)<<"\t"
       <<(currentState.wheel_velo/WHEEL_R)*currentState.torque<<"\t"
       << endl;
}

void Simulation::update_input()
{

if(currentState.external_forces[1]==0)
{
  if(ttocount==0)
  {
    if(ttocounter==0)
    {
      tto=timer;
    }
    
  }
  ttocount++;
}
else
{
  if((timer>tto)&&(ttocount!=0))
  {
    ttocount=0;
    ttocounter++;//着地したらカウントアップ
    if(ttocounter==1)
    {
      tg=timer;
    }
  }
}
 cout<<"tto="<<tto<<endl;
  cout<<"ttocount="<<ttocount<<endl;
  cout<<"ttocounter="<<ttocounter<<endl;
  cout<<"tg="<<tg<<endl;

extremum();
#ifdef AExcitation
//AngleExcitation();
//AngleExcitation_2();
AngleExcitation_f();
#endif
#ifdef Excitation
if(timer<0.05)
{
  currentState.external_forces[4]=0.1;
}
else
{
  fbExcitation();
}
#endif

BodyAngle();
PD();//角度をPD制御

StateGenerator();
Statefeedback();


//  currentState.external_forces[4]=0.0;//大腿リンクへの入力
//  currentState.external_forces[5]=0.0;//ボディリンクへの入力
}

void Simulation::fbExcitation()
{

  if(currentState.velo[3]>0)
  {
    currentState.external_forces[4]=-0.5;
  }
  else if(currentState.velo[3]<0)
  {
    currentState.external_forces[4]=0.5;
  }
  else
  {
    currentState.external_forces[4]=0;
  }
}

void Simulation::AngleExcitation()
{
  //ver1
  if(timer<STOP_TIME)
  {
    pose_ref[4]=THETA_2_START*(M_PI/180);
  }
  else 
  {
    //pose_ref[4]=THETA_2_START*(M_PI/180);
    pose_ref[4]=(lmax/2)+la*sin(wg*timer);
    // cout<<"\n"<<pose_ref[4]*(180/M_PI);
  }
  
}

void Simulation::AngleExcitation_2()
{
  if((timer<STOP_TIME)&&(currentState.external_forces[1]>0))
  {
    pose_ref[4]=THETA_2_START*(M_PI/180);
  }
  else if(currentState.external_forces[1]>0)
  {
    pose_ref[4]=(lmax/2)+la*sin(wg*timer);
  }
  else if(currentState.external_forces[1]==0)
  {
    if(ttocount==1)
    {
      tto=timer;
    }

    pose_ref[4]=(lmax/2)+la*sin(wap*timer+(wg-wap)*tto);
    cout<<"\ntto="<<tto<<endl;
    cout<<"\nttocount="<<ttocount<<endl;
    ttocount++;
  }
}

void Simulation::AngleExcitation_f()
{
  if((timer<STOP_TIME)&&(currentState.external_forces[1]>0)&&(ttocounter==0))
  {
    pose_ref[4]=THETA_2_START*(M_PI/180);
  }
  else if((timer>=STOP_TIME)&&(n<2)&&(currentState.external_forces[1]>0)&&(ttocounter==0))
  {
    pose_ref[4]=(lmax/2)+la*sin(wg*(timer-STOP_TIME));
  }
  else if((timer>=STOP_TIME)&&(n>=2)&&(currentState.external_forces[1]>0)&&(ttocounter==0))
  {
    //n=2;//add
    // pose_ref[4]=(lmax/2)+(lmax/2)*sin(wg*(timer-STOP_TIME));
    pose_ref[4]=(lmax/2)+(48)*(M_PI/180)*sin(wg*(timer-STOP_TIME));
  }
  else if((timer>=STOP_TIME)&&(n>=2)&&(currentState.external_forces[1]==0)&&(ttocounter==0))
  {
   pose_ref[4]=(lmax/2)+(5)*(M_PI/180)*sin(wap*(timer-STOP_TIME)+(wg-wap)*(tto-STOP_TIME));
   //pose_ref[4]=(lmax/2)+(lmax/2)*sin(wg*(timer-STOP_TIME));
   cout<<"離陸"<<endl;
  }
  else if((n<4)&&(ttocounter>0))
  {
    //pose_ref[4]=(lmax/2)+(5)*(M_PI/180)*sin(wap*(tg-STOP_TIME)+(wg-wap)*(tto-STOP_TIME));
    //pose_ref[4]=(lmax/2)+(45)*(M_PI/180)*sin(wap*(timer-STOP_TIME)+(wg-wap)*(tto-STOP_TIME));
    pose_ref[4]=83.125*(M_PI/180);
    cout<<"着地"<<endl;
  }
  else if((n>=4)&&(ttocounter>0))
  {
      pose_ref[4]=45*(M_PI/180);//45
    cout<<"ステップ"<<endl;
  }
  
}

int Simulation::extremum()
{
  if(timer>=STOP_TIME+delta_t)
  {
    if((timer>=STOP_TIME+delta_t)&&(timer<(STOP_TIME+2*delta_t)))
    {
      oldbane[0]=currentState.pose[3];
      cout<<"実行1"<<endl;
    }
    else if ((timer>=STOP_TIME+2*delta_t)&&(timer<(STOP_TIME+3*delta_t)))
    {
      oldbane[1]=currentState.pose[3];
      cout<<"実行2"<<endl;
    }
    else if(timer>=(STOP_TIME+3*delta_t))
    {
      if(oldbane[1]<oldbane[0])
      {
        if(currentState.pose[3]>oldbane[1])
        {
          
          n++;
          cout<<"n++"<<endl;

          if(n==3)ncount++;
          if(ncount==1)n=2;

        }
        cout<<"単調に減少"<<endl;
      }
        cout<<"実行3"<<endl;
        
        oldbane[0]=oldbane[1];
        oldbane[1]=currentState.pose[3];
    }
    cout<<"timer="<<timer<<endl;
  }
//極小値の判定
    cout<<"n="<<n<<endl;
  return n;
}

void Simulation::BodyAngle()
{
  
  //pose_ref[4]=THETA_2_START*(M_PI/180);
  pose_ref[5]=THETA_3_START*(M_PI/180);
}

void Simulation::PD()
{
  Matrix<double, 6, 1> u;
  error=pose_ref-currentState.pose;


  if(timer==0) olderror=error;

  u=P_GEIN*error+D_GEIN*(error-olderror)/delta_t;

  if(abs(u[4])>MAX_M_JOINT_TORQUE)//モータリンク入力制限
  {
    if(u[4]>0)u[4]=MAX_M_JOINT_TORQUE;
    else if(u[4]<0)u[4]=-MAX_M_JOINT_TORQUE;
  }
  if(abs(u[5])>MAX_B_JOINT_TORQUE)//ボディリンク入力制限
  {
    if(u[5]>0)u[5]=MAX_B_JOINT_TORQUE;
    else if(u[5]<0)u[5]=-MAX_B_JOINT_TORQUE;
  }

#ifdef AExcitation //角度で強制振動
  currentState.external_forces[4]=u[4];//theta2(モータ角度)
#endif

  currentState.external_forces[5]=u[5];//theta3(ボディ間のモータ角度)
  //currentState.external_forces[5]=0;//theta3(ボディ間のモータ角度)


  olderror=error;
}

void Simulation::Statefeedback()
{
  double u;

  if(timer==0) 
  {
    state<<currentState.pose(0,0),currentState.velo(0,0),currentState.theta_g,0;
    oldstate=state;
  }
  else
  {
     state<<currentState.pose(0,0),currentState.velo(0,0),currentState.theta_g,(currentState.theta_g-oldstate(2,0))/delta_t;
  }
  cout<<"theta_gref="<<state_ref(2,0)*180/M_PI;
  cout<<"theta_g="<<currentState.theta_g*180/M_PI;
    
  u=K*(state_ref-state);

  currentState.torque=u*WHEEL_R;

  oldstate=state;
  // if(abs(u)>MAX_WHEEL_TORQUE/WHEEL_R)//車輪トルク入力制限
  // {
  //   if(u>0)u=MAX_WHEEL_TORQUE/WHEEL_R;
  //   else if(u<0)u=-MAX_WHEEL_TORQUE/WHEEL_R;
  // }

  currentState.wheel_velo=state(1,0);
  currentState.wheel_velo_ref=state_ref(1,0);
}

void Simulation::StateGenerator()
{
    if(timer<STOP_TIME)
  {
    state_ref<<X_START,0.2,0,0;//1m/s
  }
  else if(n>=2)
  {
    if(ttocounter<1)
    {
      if((state(1,0)>1.0))
      {
        state_ref<<X_START,0.2,M_PI*-12.5/180,0;//-8.75
        //cout<<"\n"<<state_ref<<endl;
        cout<<"目標値"<<endl;
      }
      else
      {
        state_ref<<X_START,0.2,M_PI*0/180,0;
      }
    }
    else
    {
      state_ref<<X_START,0,M_PI*0/180,0;
    }
    
  }

  if((timer>=STOP_TIME)&&(timer<STOP_TIME+delta_t))
  {
    K<<0,-0.416116,-14.9185,-1.18899;//4.1
  }
  else if(currentState.pose[1]>=WHEEL_R+0.005)
  {
    K<<0,-0.416116,-14.9185,-1.18899;//4.1
  }

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
  _log << "time\t" 
      << "x\t" << "z\t" << "Theta_1\t" << "Theta_m\t"<<"Theta_2\t"<<"Theta_3\t"
      <<"reaction force\t"
      <<"wheel torque\t"//WHEEL MOTOR TORQUE
      << "angle2 torque\t"//BODYLINK JOINT TORQUE
      << "motor torque\t"//BODYLINK JOINT TORQUE
      << "body torque\t"//MOTORLINK JOINT TORQUE
      << "spring torque\t"
      <<"knee angle\t"
      << "Theta_2_ref\t"//目標モータ角度
      <<"CoD_z\t"
      <<"Wheel_speed\t"
      <<"Wheel_speed_ref\t"
      <<"Wheel_anglespeed\t"
      <<"Wheel_rpm\t"
      <<"Wheel_output\t"
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
            <<currentState.external_forces[1]<< "\t"
            << currentState.torque << "\t"
            <<currentState.external_forces[4] << "\t"
            <<-1*currentState.external_forces[4] << "\t"//モータリンクから見た
            << currentState.external_forces[5] << "\t"
            <<-1*currentState.pose[3]*K_SPRING<< "\t"
            <<(currentState.pose[3]+currentState.pose[4])*180/M_PI << "\t"
            <<pose_ref[4]*180/M_PI <<"\t"
            <<currentState.joint[7].z<<"\t"
            <<currentState.wheel_velo<<"\t"
            <<currentState.wheel_velo_ref<<"\t"
            <<currentState.wheel_velo/WHEEL_R<<"\t"
            <<(currentState.wheel_velo/WHEEL_R)*(30/M_PI)<<"\t"
            <<(currentState.wheel_velo/WHEEL_R)*currentState.torque<<"\t"
            << endl;
       #ifdef STOP_SIMULATION
        while( std::getchar() != '\n');
      #endif 
    
    #ifdef STOP_TIMER
    if(timer>STOP_TIME)
    {
      while( std::getchar() != '\n');   
    }
    #endif

    if (timer > finish_time) end_flag = 1;
    else end_flag =0;
  }
}

stateClass Simulation::calc_reactForce(stateClass currentState)
{
  double f_x= currentState.joint[0].x;
  double f_z = currentState.joint[0].z;  

  for(int i=0; i<6; i++)
  {
    currentState.external_forces[i]=0;//外力をリセット
  }
  currentState.get_wheel_h_min();//現在の車輪高さを取得

  double reactForce;

    if((f_z-WHEEL_R)<get_ground(f_x) )
    {//前輪高さが地面にめり込んでいるor壁にめり込んでいる
      // double torque_z=currentState.torque;
      // double hoge=abs(currentState.torque);

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

      double wheelforce =currentState.torque/WHEEL_R;
      double f_x = wheelforce;//前輪による水平力
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