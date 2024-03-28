#pragma once

#include <cmath>
#include "VecUtils.h"

const double maxSpeed=2*M_PI*5676.0/60.0;//rad/sec


const double kv=12.0/maxSpeed; // v/(rad/sec)
const double kt=12.0/2.6; // v/nm

Vec2 torqueToVoltage(Vec2 torque, Vec2 omega);
Vec2 voltageToTorque(Vec2 voltage, Vec2 omega);

const double gear_1=168;
const double gear_2=78;

