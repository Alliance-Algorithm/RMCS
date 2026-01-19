#include <eigen3/Eigen/Dense>
#include<cmath>
using namespace Eigen;



class Kinematic{

private:
    //a1 alpha1实际上表示的是MDH法中的a0 alpha0，只不过为了传参更简便，所以使用了1.看起来比较统一
    double a1=0,alpha1=0,d1=0.05985,offset1=0;
    double a2=0,alpha2=std::numbers::pi/2,d2=0,offset2=std::numbers::pi/2;
    double a3=0.41,alpha3=0,d3=0,offset3=0;
    double a4=-0.08307,alpha4=std::numbers::pi/2,d4=0.33969,offset4=0;
    double a5=0,alpha5=std::numbers::pi/2,d5=0,offset5=std::numbers::pi/2;
    double a6=0,alpha6=std::numbers::pi/2,d6=-0.0571,offset6=0;



public:
    Matrix<double, 6, 1> positive_kinematic(double theta1,double theta2,double theta3,double theta4,double theta5,double theta6){
    Matrix4d  T01,T12,T23,T34,T45,T56;
    
     T01 << cos(theta1+offset1),    -sin(theta1+offset1),    0,     a1,
     sin(theta1+offset1)*cos(alpha1), cos(theta1+offset1)*cos(alpha1), -sin(alpha1),   -sin(alpha1)*d1,
     sin(theta1+offset1)*sin(alpha1), cos(theta1+offset1)*sin(alpha1),  cos(alpha1),    cos(alpha1)*d1,
     0,     0,      0,     1;


    }









};
