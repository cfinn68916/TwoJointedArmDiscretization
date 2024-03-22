#include "ProfileUtils.h"
#include <cmath>

double calcAccel(double vel, double pos, double goal, double maxVel, double maxAccel) {
    if(fabs(pos-goal)<0.01&&fabs(vel)<0.001){
        return 0;
    }
    if(fabs(pos-goal)<0.01){
        return -vel*20;
    }
    if((pos-goal)*vel<0&&fabs(pos-goal)<0.5*vel*vel/maxAccel){
        return vel>0?-maxAccel:maxAccel;
    }
    if((pos-goal)*vel>0){
        return vel>0?-maxAccel:maxAccel;
    }
    if(fabs(vel)<maxVel){
        return pos>goal?-maxAccel:maxAccel;
    }
    return 0;
}
