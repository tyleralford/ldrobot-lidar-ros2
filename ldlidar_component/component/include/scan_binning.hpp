/**
 * @file scan_binning.hpp
 * @brief Turns one revolution of SDK points into the fields of a sensor_msgs/LaserScan.
 *
 * No ROS types, so it is unit-tested without a node (test/test_scan_binning.cpp).
 *
 * Conventions (the invariants the localizer's deskew relies on):
 *  - The sensor reports angles in degrees, clockwise from its front, and sweeps clockwise.
 *  - `rot_verse` "CCW" (the ROS standard) lays the array out with angle increasing
 *    counter-clockwise: ranges[j] looks along ROS angle angle_min + j * angle_increment, with
 *    angle_min = angle_increment, so the array ends on the sensor's zero. The sensor sweeps the
 *    other way, so beam j was measured *later* than beam j + 1: time_increment is NEGATIVE and
 *    header.stamp is the measurement time of ranges[0], the last beam of the revolution.
 *  - `rot_verse` "CW" lays the array out in the sensor's own order: angle_min = 0, time_increment
 *    positive, header.stamp the first beam's time.
 *  - Either way beam j was measured at stamp + j * time_increment, and there is no duplicate
 *    bin at 0 / 360 degrees (angle_increment = 2 pi / bins).
 *
 * The SDK stamps a point from the host arrival time of its packet, which is bunched wherever
 * several packets arrive in one read. The measurement time of a beam is therefore taken from its
 * angle and the spin rate, anchored by the median arrival-time offset over the revolution.
 */

#ifndef SCAN_BINNING_HPP_
#define SCAN_BINNING_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace ldlidar
{

/// One SDK point: angle in degrees (clockwise from the front), distance in mm, host arrival ns.
struct RawPoint
{
  float angle_deg;
  uint16_t distance_mm;
  uint8_t intensity;
  uint64_t stamp_ns;
};

struct BinningParams
{
  int bins = 0;                  ///< Fixed number of beams; 0 uses the number of points.
  bool counterclockwise = true;  ///< ROS-standard layout (see the file comment).
  float dist_scale = 0.001f;     ///< Output units per mm.
  float range_min = 0.03f;       ///< Output units; closer returns are NaN.
  float range_max = 25.0f;       ///< Output units; farther returns are NaN.
  bool crop = false;             ///< Blank sensor angles in [crop_min_deg, crop_max_deg].
  double crop_min_deg = 0.0;
  double crop_max_deg = 0.0;
  double stamp_latency_s = 0.0;  ///< Subtracted from the stamp: serial + host read latency.
};

struct BinnedScan
{
  std::vector<float> ranges;
  std::vector<float> intensities;
  float angle_min = 0.0f;
  float angle_increment = 0.0f;
  float time_increment = 0.0f;
  float scan_time = 0.0f;
  double stamp_s = 0.0;          ///< header.stamp, seconds on the clock the SDK stamped with.
};

/// Time at which the beam passed sensor angle 0 in this revolution, in seconds.
inline bool revolution_anchor_s(const std::vector<RawPoint> & points, double spin_hz, double & out)
{
  const double deg_per_s = 360.0 * spin_hz;
  std::vector<double> offsets;
  offsets.reserve(points.size());
  for (const auto & p : points) {
    if (p.stamp_ns != 0) {
      offsets.push_back(static_cast<double>(p.stamp_ns) * 1e-9 - p.angle_deg / deg_per_s);
    }
  }
  if (offsets.empty()) {
    return false;
  }
  auto mid = offsets.begin() + offsets.size() / 2;
  std::nth_element(offsets.begin(), mid, offsets.end());
  out = *mid;
  return true;
}

/// Returns false when the revolution cannot be stamped (no spin rate, no usable point stamps).
inline bool bin_scan(
  const std::vector<RawPoint> & points, double spin_hz, const BinningParams & prm, BinnedScan & out)
{
  const int n = prm.bins > 0 ? prm.bins : static_cast<int>(points.size());
  double anchor_s = 0.0;
  if (n < 2 || spin_hz <= 0.0 || !revolution_anchor_s(points, spin_hz, anchor_s)) {
    return false;
  }

  const double two_pi = 2.0 * M_PI;
  const double inc = two_pi / n;
  const double scan_time = 1.0 / spin_hz;
  const double beam_dt = scan_time / n;

  out.ranges.assign(n, std::numeric_limits<float>::quiet_NaN());
  out.intensities.assign(n, std::numeric_limits<float>::quiet_NaN());
  out.angle_increment = static_cast<float>(inc);
  out.scan_time = static_cast<float>(scan_time);

  for (const auto & p : points) {
    if (p.distance_mm == 0 && p.intensity == 0) {
      continue;  // the SDK's filter blanked it
    }
    if (prm.crop && p.angle_deg >= prm.crop_min_deg && p.angle_deg <= prm.crop_max_deg) {
      continue;
    }
    const float range = p.distance_mm * prm.dist_scale;
    if (range < prm.range_min || range > prm.range_max) {
      continue;
    }
    // Sensor-order bin, then the array position for the requested layout.
    const int k = static_cast<int>(std::lround(p.angle_deg * M_PI / 180.0 / inc)) % n;
    const int j = prm.counterclockwise ? n - 1 - k : k;
    if (std::isnan(out.ranges[j]) || range < out.ranges[j]) {
      out.ranges[j] = range;
      out.intensities[j] = p.intensity;
    }
  }

  const double latency = prm.stamp_latency_s;
  if (prm.counterclockwise) {
    out.angle_min = static_cast<float>(inc);
    out.time_increment = static_cast<float>(-beam_dt);
    out.stamp_s = anchor_s + (n - 1) * beam_dt - latency;
  } else {
    out.angle_min = 0.0f;
    out.time_increment = static_cast<float>(beam_dt);
    out.stamp_s = anchor_s - latency;
  }
  return true;
}

}  // namespace ldlidar

#endif  // SCAN_BINNING_HPP_
