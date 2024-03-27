#pragma once
#include <cmath>
#include "VecUtils.h"

namespace ArmConstants{
    const double I1 = 0.493;
    const double I2 = 0.362;
    const double m1 = 4.16;
    const double m2 = 1;
    const double r1 = 0.269;
    const double r2 = 0.243;
    const double l1 = 0.638429;
    const double l2 = 0.80645;
}
const double g = 9.8065;

Vec2 tg(Vec2 theta);
Mat coriolis(Vec2 theta, Vec2 omega);
Mat M(Vec2 theta);

Vec2 alpha(Vec2 theta, Vec2 omega, Vec2 torque);
Vec2 torques(Vec2 theta, Vec2 omega, Vec2 alpha);

Vec2 anglesToPos(Vec2 theta);
Vec2Pair posToAngles(Vec2 pos);
