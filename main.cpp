#include <iostream>
#include "cmath"
#include "vector"
#include "src/VecUtils.h"
#include "src/ArmDynamics.h"


// {theta1,theta2,omega1,omega2}
State propagate(State state, double dt){
    Vec2 a=alpha(state.pos,state.vel);
    return {state.pos+(state.vel+a*dt*0.5)*dt,state.vel+(a*dt)};
}

State euler(State state, double totalT, double dt){
    double t=0;
    State statecp=state;
    while(totalT-t>dt){
        statecp=propagate(statecp, dt);
        t+=dt;
    }
    return propagate(statecp, totalT-t);
}
double sq(double i){
    return i*i;
}

double vdiff(std::vector<double> state1,std::vector<double> state2){
    return sq(state1[0]-state2[0])+sq(state1[1]-state2[1])+sq(state1[2]-state2[2])+sq(state1[3]-state2[3]);
}


int main() {
    State st2{{1.0, 1.0}, {0, 0}};
    for (int i = 0; i < 1000; i++) {
//        std::cout << 0.02 * i  << "     " << st2.pos.a << " " << st2.pos.b << "\n";
        st2 = euler(st2, 0.02, 3E-7);
    }


    return 0;
}
