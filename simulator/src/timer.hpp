#ifndef _TIMER_HPP_
#define _TIMER_HPP_

#include <iostream>
#include <chrono>

using namespace std::chrono;
using std::chrono::duration;

class Timer
{
    private:

    public:
    Timer();
    system_clock::time_point start_time;
    system_clock::time_point end_time;
    milliseconds current_time;
    long get_current_milli();

};

#endif