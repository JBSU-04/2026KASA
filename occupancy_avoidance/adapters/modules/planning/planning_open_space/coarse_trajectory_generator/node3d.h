#pragma once
namespace apollo {namespace planning {
// Reeds-Shepp only needs continuous position, heading and initial gear.
class Node3d {double x_,y_,phi_;bool forward_=true;public:
Node3d(double x,double y,double h):x_(x),y_(y),phi_(h){}
double GetX()const{return x_;}double GetY()const{return y_;}double GetPhi()const{return phi_;}
bool GetDirec()const{return forward_;}void SetDirec(bool v){forward_=v;}
};}}
