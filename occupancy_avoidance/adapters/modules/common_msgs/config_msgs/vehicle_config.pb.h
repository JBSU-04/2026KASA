#pragma once
namespace apollo { namespace common {
class VehicleParam { double steer_=.6,ratio_=1.,base_=3.; public:
double max_steer_angle()const{return steer_;} double steer_ratio()const{return ratio_;} double wheel_base()const{return base_;}
void set_max_steer_angle(double v){steer_=v;} void set_steer_ratio(double v){ratio_=v;} void set_wheel_base(double v){base_=v;}
}; }}
