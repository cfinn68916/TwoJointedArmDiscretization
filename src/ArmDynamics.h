#pragma once
#include <cmath>
#include "VecUtils.h"

namespace ArmConstants{
    const double I1 = 1.0;
    const double I2 = 1.0;
    const double m1 = 1.0;
    const double m2 = 1.0;
    const double r1 = 0.5;
    const double r2 = 0.5;
    const double l1 = 1.0;
    const double l2 = 1.0;
}
const double g = 9.8;

Vec2 tg(Vec2 theta);
Mat coriolis(Vec2 theta, Vec2 omega);
Mat M(Vec2 theta);

Vec2 alpha(Vec2 theta, Vec2 omega);

Vec2 anglesToPos(Vec2 theta);
Vec2Pair posToAngles(Vec2 pos);
