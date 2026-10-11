// Local container adapter for compiling the original measurement utilities.
#pragma once
#include <map>
#include "modules/perception/lidar_tracking/tracker/common/tracked_object.h"
namespace apollo { namespace perception { namespace lidar {
struct MlfTrackData {
  std::map<double, TrackedObjectPtr> history;
  const std::map<double, TrackedObjectPtr>& GetHistoryObjects() const { return history; }
  std::pair<double, TrackedObjectPtr> GetHistoryObject(int index) const {
    if (history.empty()) return {0.0, nullptr};
    auto it = history.end();
    do { --it; } while (index++ < 0 && it != history.begin());
    return *it;
  }
};
using MlfTrackDataConstPtr = std::shared_ptr<const MlfTrackData>;
}}}
