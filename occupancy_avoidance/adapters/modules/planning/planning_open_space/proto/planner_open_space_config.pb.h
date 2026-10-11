#pragma once
namespace apollo {namespace planning {
// Configuration-only compatibility adapter. No protobuf/Cyber runtime.
class WarmStartConfig {double step_=.1,kappa_=1.;public:
double step_size()const{return step_;} void set_step_size(double v){step_=v;}
double traj_kappa_contraint_ratio()const{return kappa_;} void set_traj_kappa_contraint_ratio(double v){kappa_=v;}
double traj_forward_penalty()const{return 1.;} double traj_back_penalty()const{return 1.5;}
double traj_gear_switch_penalty()const{return 5.;} double traj_steer_penalty()const{return .1;}
double traj_steer_change_penalty()const{return .1;} double traj_short_length_penalty()const{return 0.;}
double traj_expected_shortest_length()const{return 0.;}
};
class PlannerOpenSpaceConfig {WarmStartConfig warm_;public: const WarmStartConfig& warm_start_config()const{return warm_;}};
}}
