#pragma once

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace smpc_lane_change {

struct Point2D {
  double x{0.0};
  double y{0.0};
};

struct Projection {
  bool valid{false};
  double s{0.0};
  double d{0.0};
  double yaw{0.0};
  double distance{std::numeric_limits<double>::infinity()};
  std::size_t segment_index{0};
};

struct LaneSample {
  bool valid{false};
  double x{0.0};
  double y{0.0};
  double s{0.0};
  double d{0.0};
  double yaw{0.0};
};

class LanePath {
 public:
  bool loadCsv(const std::string& file_path) {
    points_.clear();
    cumulative_s_.clear();
    file_path_ = file_path;

    std::ifstream file(file_path);
    if (!file.is_open()) return false;

    std::string line;
    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string cell;
      std::vector<std::string> cells;
      while (std::getline(ss, cell, ',')) cells.push_back(cell);
      if (cells.size() < 3) continue;

      try {
        // MPC waypoint CSV: index,x,y,mission_state,heading,curvature
        points_.push_back({std::stod(cells[1]), std::stod(cells[2])});
      } catch (...) {
        continue;
      }
    }

    if (points_.size() < 2) return false;
    cumulative_s_.resize(points_.size(), 0.0);
    for (std::size_t i = 1; i < points_.size(); ++i) {
      cumulative_s_[i] = cumulative_s_[i - 1] +
          std::hypot(points_[i].x - points_[i - 1].x,
                     points_[i].y - points_[i - 1].y);
    }
    return true;
  }

  // Extend only the in-memory lane geometry behind the first CSV point.
  // Original CSV files and their waypoint indices remain untouched.  This is
  // needed when an adjacent lane starts a few metres later than the current
  // lane: objects beside ego must still project onto that lane before ego
  // reaches its index 0.
  bool prependLinearExtension(double extension_m, double spacing_m = 0.1) {
    if (points_.size() < 2 || extension_m <= 1e-3) return false;
    spacing_m = std::max(0.05, spacing_m);

    // Use a multi-point chord when possible so one noisy first CSV segment
    // does not rotate the virtual lane extension.
    std::size_t tangent_end = 1;
    while (tangent_end + 1 < points_.size() && tangent_end < 30 &&
           std::hypot(points_[tangent_end].x - points_.front().x,
                      points_[tangent_end].y - points_.front().y) < 2.0) {
      ++tangent_end;
    }
    const double dx = points_[tangent_end].x - points_.front().x;
    const double dy = points_[tangent_end].y - points_.front().y;
    const double norm = std::hypot(dx, dy);
    if (norm < 1e-6) return false;
    const double ux = dx / norm;
    const double uy = dy / norm;

    const int count = std::max(1, static_cast<int>(std::ceil(extension_m / spacing_m)));
    std::vector<Point2D> extended;
    extended.reserve(static_cast<std::size_t>(count) + points_.size());
    for (int i = count; i >= 1; --i) {
      const double back = std::min(extension_m, static_cast<double>(i) * spacing_m);
      extended.push_back({points_.front().x - back * ux,
                          points_.front().y - back * uy});
    }
    extended.insert(extended.end(), points_.begin(), points_.end());
    points_.swap(extended);
    rebuildCumulativeDistance();
    return true;
  }

  // Match the path publisher's left-offset prelude before the physical CSV.
  // Reject unrelated/poorly aligned joins without mutating the original lane.
  bool prependOffsetExtension(const LanePath& source, double extension_m,
                              double offset_m, double max_error, double max_yaw) {
    if (source.points_.size() < 3 || points_.size() < 2 || extension_m <= 0)
      return false;
    std::vector<Point2D> shifted;
    std::vector<double> headings;
    for (std::size_t i = 0; i < source.points_.size(); ++i) {
      const auto a = i + 1 < source.points_.size() ? i : i - 1;
      const auto b = a + 1;
      const double yaw = std::atan2(source.points_[b].y-source.points_[a].y,
                                    source.points_[b].x-source.points_[a].x);
      headings.push_back(yaw);
      shifted.push_back({source.points_[i].x-std::sin(yaw)*offset_m,
                         source.points_[i].y+std::cos(yaw)*offset_m});
    }
    std::size_t join = 0;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i=0; i<shifted.size(); ++i) {
      const double d=std::hypot(shifted[i].x-points_[0].x, shifted[i].y-points_[0].y);
      if (d<best) { best=d; join=i; }
    }
    const double yaw=std::atan2(points_[1].y-points_[0].y, points_[1].x-points_[0].x);
    if (join<2 || best>max_error ||
        std::abs(std::atan2(std::sin(yaw-headings[join]),
                           std::cos(yaw-headings[join])))>max_yaw) return false;
    std::size_t begin=join;
    double arc=0;
    while (begin>0 && arc<extension_m) {
      arc+=std::hypot(shifted[begin].x-shifted[begin-1].x,
                      shifted[begin].y-shifted[begin-1].y);
      --begin;
    }
    std::vector<Point2D> extended(shifted.begin()+begin, shifted.begin()+join+1);
    extended.insert(extended.end(),points_.begin(),points_.end());
    points_.swap(extended);
    rebuildCumulativeDistance();
    return true;
  }

  Projection project(double qx, double qy) const {
    Projection best;
    if (points_.size() < 2) return best;

    double best_d2 = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < points_.size(); ++i) {
      const double vx = points_[i + 1].x - points_[i].x;
      const double vy = points_[i + 1].y - points_[i].y;
      const double len2 = vx * vx + vy * vy;
      if (len2 < 1e-10) continue;

      const double rx = qx - points_[i].x;
      const double ry = qy - points_[i].y;
      const double t = std::clamp((rx * vx + ry * vy) / len2, 0.0, 1.0);
      const double px = points_[i].x + t * vx;
      const double py = points_[i].y + t * vy;
      const double ex = qx - px;
      const double ey = qy - py;
      const double d2 = ex * ex + ey * ey;

      if (d2 < best_d2) {
        best_d2 = d2;
        const double seg_len = std::sqrt(len2);
        const double cross = vx * ey - vy * ex;
        best.valid = true;
        best.s = cumulative_s_[i] + t * seg_len;
        best.d = (cross >= 0.0 ? 1.0 : -1.0) * std::sqrt(d2);
        best.yaw = std::atan2(vy, vx);
        best.distance = std::sqrt(d2);
        best.segment_index = i;
      }
    }
    return best;
  }

  LaneSample sample(double query_s, double lateral_d = 0.0) const {
    LaneSample out;
    if (points_.size() < 2 || cumulative_s_.size() != points_.size()) return out;

    const double clamped_s = std::clamp(query_s, 0.0, length());
    auto upper = std::lower_bound(cumulative_s_.begin(), cumulative_s_.end(), clamped_s);
    std::size_t i = 0;
    if (upper == cumulative_s_.begin()) {
      i = 0;
    } else if (upper == cumulative_s_.end()) {
      i = points_.size() - 2;
    } else {
      i = static_cast<std::size_t>(std::distance(cumulative_s_.begin(), upper) - 1);
      i = std::min(i, points_.size() - 2);
    }

    const double vx = points_[i + 1].x - points_[i].x;
    const double vy = points_[i + 1].y - points_[i].y;
    const double seg_len = std::hypot(vx, vy);
    if (seg_len < 1e-9) return out;

    const double t = std::clamp((clamped_s - cumulative_s_[i]) / seg_len, 0.0, 1.0);
    const double cx = points_[i].x + t * vx;
    const double cy = points_[i].y + t * vy;
    const double yaw = std::atan2(vy, vx);
    const double nx = -std::sin(yaw);
    const double ny = std::cos(yaw);

    out.valid = true;
    out.x = cx + lateral_d * nx;
    out.y = cy + lateral_d * ny;
    out.s = clamped_s;
    out.d = lateral_d;
    out.yaw = yaw;
    return out;
  }

  bool empty() const { return points_.empty(); }
  double length() const { return cumulative_s_.empty() ? 0.0 : cumulative_s_.back(); }
  const std::vector<Point2D>& points() const { return points_; }
  const std::string& filePath() const { return file_path_; }

 private:
  void rebuildCumulativeDistance() {
    cumulative_s_.assign(points_.size(), 0.0);
    for (std::size_t i = 1; i < points_.size(); ++i) {
      cumulative_s_[i] = cumulative_s_[i - 1] +
          std::hypot(points_[i].x - points_[i - 1].x,
                     points_[i].y - points_[i - 1].y);
    }
  }

  std::string file_path_;
  std::vector<Point2D> points_;
  std::vector<double> cumulative_s_;
};

inline double wrapToPi(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace smpc_lane_change
