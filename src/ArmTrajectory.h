#pragma once

#include <vector>
#include "VecUtils.h"

struct TrajState{
    Vec2 pos;
    Vec2 vel;
    Vec2 accel;
};
Vec2 interpolate(Vec2 start,Vec2 end, double t);

class Trajectory{
public:
    std::vector<Vec2> waypoints;
    double time;

    Trajectory(const std::vector<Vec2> &waypoints, double time) : waypoints(waypoints), time(time) {};
    TrajState getAtTime(double t);
    static Trajectory fromTwoPoints(Vec2 start, Vec2 end, double time, int innerPoints);
};
