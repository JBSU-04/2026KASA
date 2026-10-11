// Apollo's math core needs only logging/check macros, not Cyber runtime.
#pragma once
#include <stdexcept>
#include <sstream>
namespace occupancy_apollo_adapter {
struct SilentLog { template<class T> SilentLog& operator<<(const T&){return *this;} };
struct Check {
  bool ok;const char* expression;
  Check(bool valid,const char*text):ok(valid),expression(text){}
  template<class T> Check& operator<<(const T&){return *this;}
  ~Check() noexcept(false){if(!ok)throw std::runtime_error(expression);}
};
template<class T> T* notnull(T*p){if(!p)throw std::runtime_error("Apollo null pointer");return p;}
}
#define AERROR occupancy_apollo_adapter::SilentLog()
#define ADEBUG occupancy_apollo_adapter::SilentLog()
#define AWARN occupancy_apollo_adapter::SilentLog()
#define CHECK_EQ(a,b) occupancy_apollo_adapter::Check((a)==(b),"Apollo CHECK_EQ " #a " " #b)
#define CHECK_GE(a,b) occupancy_apollo_adapter::Check((a)>=(b),"Apollo CHECK_GE " #a " " #b)
#define CHECK_NOTNULL(p) occupancy_apollo_adapter::notnull(p)

#define AINFO occupancy_apollo_adapter::SilentLog()
#define AINFO_IF(x) occupancy_apollo_adapter::SilentLog()
#define ACHECK(x) occupancy_apollo_adapter::Check(bool(x), "Apollo ACHECK " #x)
