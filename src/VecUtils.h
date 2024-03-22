#pragma once

class Vec2{
public:
    double a;
    double b;
    Vec2 operator+(Vec2 rhs);
    Vec2 operator-(Vec2 rhs);
    Vec2 operator-();
    Vec2 operator*(double rhs);
    Vec2(double a, double b) : a(a), b(b) {}
};

struct State{
    Vec2 pos;
    Vec2 vel;
};

class Mat{
public:
    double a;
    double b;
    double c;
    double d;
    Mat operator+(Mat rhs);
    Mat inv();
    Mat operator-(Mat rhs);
    Mat operator-();
    Vec2 operator*(Vec2 rhs);
    Mat(double a, double b, double c, double d) : a(a), b(b), c(c), d(d) {}
};
