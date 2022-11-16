#include "RungeKutta.hpp"

stateClass RungeKutta(stateClass state, stateClass(*func)(stateClass)){
    //funcは返り値stateClass 戻り値stateClassの関数のアドレスを格納する関数ポインタ
    stateClass state_org = state;
    stateClass k1, k2, k3, k4;
   

    k1.velo = delta_t * (*func)(state).velo;//eq_differentialを実行
    k1.pose = delta_t * (*func)(state).pose;

    state.velo = state_org.velo +0.5* k1.velo;
    state.pose = state_org.pose + 0.5 * k1.pose;
    k2.velo = delta_t * (*func)(state).velo;
    k2.pose = delta_t * (*func)(state).pose;

    state.velo = state_org.velo +0.5* k2.velo;
    state.pose = state_org.pose + 0.5 * k2.pose;
    k3.velo = delta_t * (*func)(state).velo;
    k3.pose = delta_t * (*func)(state).pose;

    state.velo = state_org.velo + k3.velo;
    state.pose = state_org.pose + k3.pose;
    k4.velo = delta_t * (*func)(state).velo;
    k4.pose = delta_t * (*func)(state).pose;

    
    state.velo = state_org.velo + (k1.velo + 2*k2.velo + 2*k3.velo + k4.velo) /6 ;
    state.pose = state_org.pose + (k1.pose + 2*k2.pose + 2*k3.pose + k4.pose) /6 ;

    state.accel = (*func)(state).velo;

    return state;
}