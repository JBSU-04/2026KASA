// Header location adapter only; OSQP and Apollo sources are unmodified.
#pragma once
#include <osqp.h>
#include <stdexcept>
// Apollo's original two-argument API predates OSQP 0.6. This compatibility
// overload is needed to compile it; the standalone wrapper below uses 0.6
// directly and owns all allocations, including the upstream CSC containers.
inline OSQPWorkspace* osqp_setup(OSQPData*data,OSQPSettings*settings){
  OSQPWorkspace*w=nullptr;if(::osqp_setup(&w,data,settings))throw std::runtime_error("OSQP setup failed");return w;
}
