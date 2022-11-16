#include "timer.hpp"

Timer::Timer(){
    start_time = system_clock::now();
    end_time = system_clock::now();
}

long Timer::get_current_milli(){
    end_time = system_clock::now();
    current_time = duration_cast<milliseconds>(end_time-start_time);

    return current_time.count();
}