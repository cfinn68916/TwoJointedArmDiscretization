// hi
#include <iostream>
#include "cmath"
#include "vector"
// a b
// c d
// [a,b,c,d]
// a
// b
// [a,b]


struct Mat{
    double a=0;
    double b=0;
    double c=0;
    double d=0;
};

struct Vec2{
    double a=0;
    double b=0;
};

struct Vec4{
    double a=0;
    double b=0;
    double c=0;
    double d=0;
};
Vec4 ccat(Vec2 a, Vec2 b){
    return {a.a,a.b,b.a,b.b};
}



double I1=1.0;
double I2=1.0;
double m1=1.0;
double m2=1.0;
double r1=0.5;
double r2=0.5;
double g=9.8;
double l1=1.0;


Mat matInv(Mat inp){
    double det=inp.a*inp.d-inp.b*inp.c;
    return {inp.d/det,-inp.b/det,-inp.c/det,inp.a/det};
}

Vec2 vecMul(Mat mat, Vec2 vec){
    return {mat.a*vec.a+mat.b*vec.b,mat.c*vec.a+mat.d*vec.b};
}

Vec2 plus(Vec2 lhs, Vec2 rhs){
    return {lhs.a+rhs.a,lhs.b+rhs.b};
}
Vec2 um(Vec2 lhs){
    return {-lhs.a,-lhs.b};
}

double c1(Vec2 theta){
    return cos(theta.a);
}
double s1(Vec2 theta){
    return sin(theta.a);
}
double c12(Vec2 theta){
    return cos(theta.a+theta.b);
}
double s12(Vec2 theta){
    return sin(theta.a+theta.b);
}

double c2(Vec2 theta){
    return cos(theta.b);
}
double s2(Vec2 theta){
    return sin(theta.b);
}

double mc1=(m1*r1+m2*l1)*g;
double mc2=m2*r2*g;
Vec2 tg(Vec2 theta){
    double mcc12=c12(theta);
    return {mc1*c1(theta)+mc2*mcc12,mc2*mcc12};
}
double mcs2preq=m2*l1*r2;
Mat corriolis(Vec2 theta, Vec2 omega){
    double mcs2=s2(theta)*mcs2preq;
    return {-mcs2*omega.b,-mcs2*(omega.a+omega.b),mcs2*omega.a,0};
}
double mc4=m2*r2*r2+I2;
double mc5=m2*l1*r2;
double mc6=m1*r1*r1;
double mc7=m2*l1*l1;
double mc467ISum= mc6 + mc7 + mc4 + I1;

Mat M(Vec2 theta){
    double mcc2=c2(theta)*mc5;
    double mcc2p4=mc4+mcc2;
    return {
            mc467ISum + 2 * mcc2,
            mcc2p4,
            mcc2p4,
            mc4
    };
}

Vec2 alpha(Vec2 theta, Vec2 omega){
    return vecMul(matInv(M(theta)),um(plus(vecMul(corriolis(theta,omega),omega),tg(theta))));
}



// {theta1,theta2,omega1,omega2}
Vec4 propagate(Vec4 state, double dt){
    Vec2 a=alpha({state.a,state.b},{state.c,state.d});
    return {state.a+dt*(state.c+a.a*dt/2),state.b+dt*(state.d+a.b*dt/2),state.c+a.a*dt,state.d+a.b*dt};
}

Vec4 euler(Vec4 state, double totalT, double dt){
    double t=0;
    Vec4 statecp=state;
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
    Vec4 st2{1.0, 1.0, 0, 0};
    for (int i = 0; i < 1000; i++) {
//        std::cout << 0.02 * i  << "     " << st2.a << " " << st2.b << "\n";
        st2 = euler(st2, 0.02, 1E-9);
    }


    return 0;
}