#include <cmath>
#include "VecUtils.h"

Vec2 Vec2::operator+(Vec2 rhs) {
    return {a+rhs.a, b+rhs.b};
}

Vec2 Vec2::operator-(Vec2 rhs) {
    return {a-rhs.a, b-rhs.b};
}

Vec2 Vec2::operator-() {
    return {-a, -b};
}

Vec2 Vec2::operator*(double rhs) {
    return {a*rhs, b*rhs};
}

double Vec2::Norm() {
    return hypot(a,b);
}

Mat Mat::operator+(Mat rhs) {
    return {a+rhs.a, b+rhs.b, c+rhs.c, d+rhs.d};
}

Vec2 Mat::operator*(Vec2 rhs) {
    return {a*rhs.a+b*rhs.b, c*rhs.a+d*rhs.b};
}

Mat Mat::operator-(Mat rhs) {
    return {a-rhs.a, b-rhs.b, c-rhs.c, d-rhs.d};
}

Mat Mat::operator-() {
    return {-a, -b, -c, -d};
}

Mat Mat::inv() {
    double det=a*d-b*c;
    return {d/det,-b/det,-c/det,a/det};
}
