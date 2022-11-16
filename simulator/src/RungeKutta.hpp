#ifndef _RungeKutta_H_
#define _RungeKutta_H_

#include "model.hpp"
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>

extern double delta_t;
stateClass RungeKutta(stateClass state, stateClass(*func)(stateClass state));

#endif //RungeKutta