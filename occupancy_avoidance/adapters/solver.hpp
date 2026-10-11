#pragma once
#include "modules/planning/planning_base/math/piecewise_jerk/piecewise_jerk_path_problem.h"
#include <cstdlib>
namespace occupancy_avoidance {
// Reuse original Apollo formulation and OSQP settings. Only API adaptation and
// resource ownership differ, avoiding a per-cycle leak of upstream containers.
class StandalonePathProblem:public apollo::planning::PiecewiseJerkPathProblem {
 public:
  using apollo::planning::PiecewiseJerkPathProblem::PiecewiseJerkPathProblem;
  int status=0;
  bool Optimize(int iterations){
    OSQPData*data=static_cast<OSQPData*>(std::calloc(1,sizeof(OSQPData)));
    if(!data)return false;
    auto release=[&](){if(data->P&&data->A){FreeData(data);c_free(data->P);c_free(data->A);}c_free(data);};
    if(FormulateProblem(data)){release();return false;}
    OSQPSettings*settings=SolverDefaultSettings();settings->max_iter=iterations;
    settings->verbose=std::getenv("OCCUPANCY_QP_DEBUG")!=nullptr;
    OSQPWorkspace*work=nullptr;
    if(::osqp_setup(&work,data,settings)){release();c_free(settings);return false;}
    osqp_solve(work);status=work->info->status_val;bool ok=status==OSQP_SOLVED;
    if(ok&&work->solution){x_.resize(num_of_knots_);dx_.resize(num_of_knots_);ddx_.resize(num_of_knots_);
      for(size_t i=0;i<num_of_knots_;++i){x_[i]=work->solution->x[i]/scale_factor_[0];dx_[i]=work->solution->x[num_of_knots_+i]/scale_factor_[1];ddx_[i]=work->solution->x[2*num_of_knots_+i]/scale_factor_[2];}}
    osqp_cleanup(work);release();c_free(settings);return ok;
  }
};
}
