#include <stdexcept>
#include "ArmTrajectory.h"
#include <cmath>

Vec2 interpolate(Vec2 start,Vec2 end, double t){
    return start+((end-start)*t);
}


Trajectory Trajectory::fromTwoPoints(Vec2 start, Vec2 end, double time, int innerPoints) {
    std::vector<Vec2> pts{};
    pts.push_back(start);
    for(int i=0;i<innerPoints;i++){
        pts.push_back(interpolate(start,end,(i+1.0)/(innerPoints+1.0)));
    }
    pts.push_back(end);
    return {pts,time};
}
int clamp(int v, int low, int high){
    return (v<low)?low:((v>high)?high:v);
}

TrajState Trajectory::getAtTime(double t) {
    if(t<0){
        throw std::invalid_argument("time is not within range");
    }
    if(t>time){
        throw std::invalid_argument("time is not within range");
    }
    if(t==0){
        return {waypoints[0],{0,0},{0,0}};
    }
    if(t==time){
        return {waypoints.back(),{0,0},{0,0}};
    }
    double dt=time/(waypoints.size()-1.0);
    double interpolant=(fmod(t,dt))/dt;

    int prevIndex=(int) floor(t/dt);
    int nextIndex=(int) ceil(t/dt);

    if(nextIndex==prevIndex){
        nextIndex++;
    }
    int prevPrevIndex=prevIndex-1;
    int nextNextIndex=nextIndex+1;
    prevPrevIndex= clamp(prevPrevIndex,0,waypoints.size()-1);
    prevIndex= clamp(prevIndex,0,waypoints.size()-1);
    nextIndex= clamp(nextIndex,0,waypoints.size()-1);
    nextNextIndex= clamp(nextNextIndex,0,waypoints.size()-1);

    Vec2 pos= interpolate(waypoints[prevIndex],waypoints[nextIndex],interpolant);
    Vec2 vel=(waypoints[nextIndex]-waypoints[prevIndex])*(1.0/dt);
    Vec2 accel{0,0};
    if(interpolant<0.5){
        Vec2 prevVel=(waypoints[prevIndex]-waypoints[prevPrevIndex])*(1.0/dt);
        accel=(vel-prevVel)*(1.0/dt);
    }else{
        Vec2 nextVel=(waypoints[nextNextIndex]-waypoints[nextIndex])*(1.0/dt);
        accel=(nextVel-vel)*(1.0/dt);
    }
    return {pos,vel,accel};
}
