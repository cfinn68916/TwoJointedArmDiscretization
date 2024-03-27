#include "ArmDynamics.h"

using namespace ArmConstants;

Vec2 alpha(Vec2 theta, Vec2 omega) {
    return -(M(theta).inv()*((coriolis(theta,omega)*omega)+tg(theta)));
}

Mat M(Vec2 theta) {
    double mc4= m2 * r2 * r2 + I2;
    double mc5= m2 * l1 * r2;
    double mc6= m1 * r1 * r1;
    double mc7= m2 * l1 * l1;
    double mc467ISum= mc6 + mc7 + mc4 + I1;
    double mcc2=cos(theta.b)*mc5;
    double mcc2p4=mc4+mcc2;
    return {
            mc467ISum + 2 * mcc2,
            mcc2p4,
            mcc2p4,
            mc4
    };
}

Mat coriolis(Vec2 theta, Vec2 omega) {
    double mcs2preq= m2 * l1 * r2;
    double mcs2=sin(theta.b)*mcs2preq;
    return {-mcs2*omega.b,-mcs2*(omega.a+omega.b),mcs2*omega.a,0};
}

Vec2 tg(Vec2 theta) {double mc1= (m1 * r1 + m2 * l1) * g;
    double mc2= m2 * r2 * g;
    double mcc12=cos(theta.a+theta.b);
    return {mc1*cos(theta.a)+mc2*mcc12,mc2*mcc12};
}

Vec2 anglesToPos(Vec2 theta){
    return {l1*cos(theta.a)+l2*cos(theta.a+theta.b),l1*sin(theta.a)+l2*sin(theta.a+theta.b)};
}

Vec2Pair posToAngles(Vec2 pos){
    double theta_2=acos((pos.a*pos.a+pos.b*pos.b-l1*l1-l2*l2)/(2*l1*l2));
    double atanxy= atan2(pos.b,pos.a);
    return {{atanxy-atan((l2*sin(-theta_2))/(l1+l2*cos(-theta_2))),-theta_2},{atanxy-atan((l2*sin(theta_2))/(l1+l2*cos(theta_2))),theta_2}};
}
