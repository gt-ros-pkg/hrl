#include <cmath>
#include <ros/ros.h>

using namespace std;

#define SQ(x) ((x) * (x))
#define PI 3.14159265

class Ellipsoid {
private:
    double A_, B_, E_;
public:
    Ellipsoid(double A, double B) : A_(A), B_(B) {
        E_ = sqrt(fabs(SQ(A_) - SQ(B_))) / A_;
    }

    void cartToEllipsoidal(double x, double y, double z, double& lat, double& lon, double& height);
    void ellipsoidalToCart(double lat, double lon, double height, double& x, double& y, double& z);
    void mollweideProjection(double lat, double lon, double& x, double& y);
};

void Ellipsoid::cartToEllipsoidal(double x, double y, double z, double& lat, double& lon, double& height) {
    lon = atan2(y, x);
    if(lon < 0)
        printf("*");
    //lon += PI;
    double a = A_ * E_;
    double p = sqrt(SQ(x) + SQ(y));
    double d1 = sqrt(SQ(p + a) + SQ(z));
    double d2 = sqrt(SQ(p - a) + SQ(z));
    double cosh_height = (d1 + d2) / (2 * a);
    height = log(cosh_height + sqrt(SQ(cosh_height) - 1));
    lat = acos((d1 - d2) / (2 * a));
    if(z > 0)
        lat = fabs(lat);
    else
        lat = -fabs(lat);
}

void Ellipsoid::ellipsoidalToCart(double lat, double lon, double height, double& x, double& y, double& z) {
    double a = A_ * E_;
    x = a * sinh(height) * sin(lat) * cos(lon);
    y = a * sinh(height) * sin(lat) * sin(lon);
    z = a * cosh(height) * cos(lat);
}

void Ellipsoid::mollweideProjection(double lat, double lon, double& x, double& y) {
    double a = A_;
    double b = A_ * (1 - SQ(E_)) / (PI * E_) * (log((1 + E_) / (1 - E_)) + 2 * E_ / (1 - SQ(E_)));
    double Sl = sin(lat);
    double k = PI * ( (log((1 + E_ * Sl) / (1 - E_ * Sl)) + 2 * E_ * Sl / (1 - SQ(E_) * SQ(Sl))) /
                      (log((1 + E_)      / (1 - E_))      + 2 * E_      / (1 - SQ(E_))));
    double t = lat;
    double diff_val = 10000.0;
    while(fabs(diff_val) > 0.00001) {
        diff_val = - ( 2 * t + sin(2 * t) - k) / (2 + 2 * cos(2 * t));
        t += diff_val;
    }
    x = a * lon * cos(t);
    y = b * sin(t);
}
