#include <smpc_lane_change/lane_path.hpp>
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <iostream>
int main(int argc, char** argv) {
  assert(argc == 2);
  smpc_lane_change::LanePath source, target;
  assert(source.loadCsv(std::string(argv[1])+"/0.csv"));
  assert(target.loadCsv(std::string(argv[1])+"/1.csv"));
  const double original=target.length();
  assert(target.prependOffsetExtension(source,80,3.5,1.5,.35));
  assert(target.length()>original+70);
  // Recorded M6 entry: the virtual target lane must remain one lane away.
  const auto projected=target.project(302644.649,4124461.424);
  assert(projected.valid && projected.distance>2.5 && projected.distance<4.5);
  smpc_lane_change::LanePath invalid;
  assert(invalid.loadCsv(std::string(argv[1])+"/1.csv"));
  assert(!invalid.prependOffsetExtension(source,80,100,1.5,.35));
  assert(invalid.length()==original);
  std::cout << "CSV prelude regression passed; entry distance=" << projected.distance << '\n';
}
