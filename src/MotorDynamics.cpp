#include "MotorDynamics.h"

Vec2 torqueToVoltage(Vec2 torque, Vec2 omega){
    return {kv*omega.a*gear_1+kt*torque.a/gear_1,kv*omega.b*gear_2+kt*torque.b/gear_2};
}
Vec2 voltageToTorque(Vec2 voltage, Vec2 omega){
    Vec2 torqueVolts= voltage-torqueToVoltage({0,0},omega);
    return {gear_1*torqueVolts.a/kt,gear_2*torqueVolts.b/kt};
}
