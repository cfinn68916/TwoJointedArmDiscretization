#include <iostream>
#include "cmath"
#include "src/VecUtils.h"
#include "src/ArmDynamics.h"
#include "src/ProfileUtils.h"
#include <fstream>
#include <sstream>


State propagate(State state, Vec2 u, double dt){
    Vec2 a=alpha(state.pos,state.vel, u);
    return {state.pos+(state.vel+a*dt*0.5)*dt,state.vel+(a*dt)};
}

State euler(State state, double totalT, double dt, Vec2 u){
    double t=0;
    State statecp=state;
    while(totalT-t>dt){
        statecp=propagate(statecp,u, dt);
        t+=dt;
    }
    return propagate(statecp,u, totalT-t);
}

Vec2 torquesForGoal(State s){
    double goal_a=-M_PI;
    double goal_b=-M_PI/4;
    return torques(s.pos,s.vel,{calcAccel(s.vel.a,s.pos.a,goal_a,1,2),calcAccel(s.vel.b,s.pos.b,goal_b,1,2)});
}



std::string toTwoHex(int inp){
    std::stringstream stre{};
    stre << std::hex << inp;
    std::string res(stre.str());
    while(res.size()<2){
        res.insert(res.cbegin(),'0');
    }
    return res;
}
std::string toTwoHex(double inp){
    return toTwoHex((int) inp);
}

Vec2 angleToSvg(Vec2 theta){
    Vec2 pos= anglesToPos(theta);
    return {pos.a*100+300,300-100*pos.b};
}


int main() {

    std::ofstream f;
    f.open("out.svg");
    f << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n"
         "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\" \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n"
         "<svg width=\"4000\" height=\"4000\" viewBox=\"-70.5 -70.5 4000 4000\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\">";


    f << "<line x1=\"" << 280 << "\" y1=\"" << 300 << "\" x2=\"" << 320<< "\" y2=\"" << 300<< "\" stroke=\"#0000FF\" stroke-width=\"1\" />";



    State st2{{2.0, 2.0}, {0, 0}};

    f << "<line x1=\"" << 300 << "\" y1=\"" << 300 << "\" x2=\"" << 300+100*ArmConstants::l1*cos(st2.pos.a)<< "\" y2=\"" << 300-100*ArmConstants::l2*sin(st2.pos.a)<< "\" stroke=\"#FF00FF\" stroke-width=\"1\" />";
    f << "<line x1=\"" << 300+100*ArmConstants::l1*cos(st2.pos.a) << "\" y1=\"" << 300-100*ArmConstants::l2*sin(st2.pos.a) << "\" x2=\"" << 300+100*(ArmConstants::l1*cos(st2.pos.a)+ArmConstants::l2*cos(st2.pos.a+st2.pos.b)) << "\" y2=\"" << 300-100*(ArmConstants::l1*sin(st2.pos.a)+ArmConstants::l2*sin(st2.pos.a+st2.pos.b)) << "\" stroke=\"#FF00FF\" stroke-width=\"1\" />";


    State st3{{2.0, 2.0}, {0, 0}};
    for (int i = 0; i < 300; i++) {
//        std::cout << 0.02 * i  << "     " << st2.pos.a << " " << st2.pos.b << "\n";
        st2 = euler(st2, 0.02, 1E-6, torquesForGoal(st2));

        f << "<line x1=\"" << angleToSvg(st3.pos).a << "\" y1=\"" << angleToSvg(st3.pos).b << "\" x2=\"" << angleToSvg(st2.pos).a<< "\" y2=\"" << angleToSvg(st2.pos).b << "\" stroke=\"#"
          << toTwoHex(256.0*i/300.0) <<toTwoHex((256.0*(300.0-i)/300.0)) << "00\" stroke-width=\"1\" />";

        st3=st2;
    }

    f << "<line x1=\"" << 300 << "\" y1=\"" << 300 << "\" x2=\"" << 300+100*ArmConstants::l1*cos(st2.pos.a)<< "\" y2=\"" << 300-100*ArmConstants::l2*sin(st2.pos.a)<< "\" stroke=\"#00FFFF\" stroke-width=\"1\" />";
    f << "<line x1=\"" << 300+100*ArmConstants::l1*cos(st2.pos.a) << "\" y1=\"" << 300-100*ArmConstants::l2*sin(st2.pos.a) << "\" x2=\"" << 300+100*(ArmConstants::l1*cos(st2.pos.a)+ArmConstants::l2*cos(st2.pos.a+st2.pos.b)) << "\" y2=\"" << 300-100*(ArmConstants::l1*sin(st2.pos.a)+ArmConstants::l2*sin(st2.pos.a+st2.pos.b)) << "\" stroke=\"#00FFFF\" stroke-width=\"1\" />";


    f << "</svg>";
    f.close();



    return 0;
}
