#ifndef _VISUALIZE_HPP_
#define _VISUALIZE_HPP_

#include "visualize.hpp"
#include "linkconf_ver2.hpp"
#include "RungeKutta.hpp"
#include "simulation.hpp"
#include "timer.hpp"
#include "model.hpp"
#include <iostream>
#include <string>
#include <chrono>
#include <unistd.h>
#include <cmath>
#include <typeinfo>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
#include <GL/glut.h>

using namespace std::chrono;
using std::string;

class Visualize
{   
    private:
    static double ratio;
    static double time_current;
    static double time_pre;
    static Simulation* v_simulation;
    static stateClass* state_visu;
    static void display();
    static void printSimulation(void);
    static void printGround();
    static void DrawString(string str, int w, int h, int x0, int y0);
    static void printCircle(int n,double x,double y,double r); 
    static void printArc(double x,double y,double r,double rad);
    static void printWheelTorque(double x, double y, double torque);
    void init(void);
    static void idle(void);

    public:
    Visualize();
    int visualize(int argc, char *argv[], Simulation& _simulation);
};

#endif 