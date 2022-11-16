#include"visualize.hpp"
#include"simulation.hpp"
#include"model.hpp"
#include"linkconf_ver2.hpp"
#include"timer.hpp"
#include <iostream>
#include <thread>   

Simulation _simulation;
Visualize _visualize;
Timer system_time;
 
int main(int argc, char *argv[]){
    stateClass state;//ロボットの状態変数
    std::cout << "---START---\n" << std::endl;
    std::thread th_visu(&Visualize::visualize,&_visualize ,argc, argv,  std::ref(_simulation) );
    std::thread th_simu(&Simulation::simu_loop, &_simulation, std::ref(state));
    th_visu.join();
    th_simu.join();
    
    return 0;
}