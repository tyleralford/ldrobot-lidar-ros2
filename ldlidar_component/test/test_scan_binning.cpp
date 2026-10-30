#include <gtest/gtest.h>

#include <cmath>

#include "scan_binning.hpp"

using ldlidar::BinnedScan;
using ldlidar::BinningParams;
using ldlidar::RawPoint;

namespace
{
constexpr double kSpinHz = 10.0;
constexpr double kT0 = 1000.0;  // s, the revolution's time at sensor angle 0

// One revolution of `n` evenly spaced points; each point's range encodes its sensor angle.
std::vector<RawPoint> revolution(int n, double arrival_bunching_s = 0.0)
{
  std::vector<RawPoint> pts;
  for (int i = 0; i < n; ++i) {
    const double angle = 360.0 * i / n;
    const double t = kT0 + angle / (360.0 * kSpinHz) + arrival_bunching_s * ((i % 7) - 3) / 3.0;
    pts.push_back(
      {static_cast<float>(angle), static_cast<uint16_t>(1000 + i), 200,
        static_cast<uint64_t>(t * 1e9)});
  }
  return pts;
}

BinningParams params(int bins, bool ccw)
{
  BinningParams p;
  p.bins = bins;
  p.counterclockwise = ccw;
  return p;
}
}  // namespace

TEST(ScanBinning, CcwArrayEndsOnSensorZeroAndTimeRunsBackwards)
{
  BinnedScan s;
  ASSERT_TRUE(ldlidar::bin_scan(revolution(2160), kSpinHz, params(2160, true), s));
  ASSERT_EQ(s.ranges.size(), 2160u);
  // ranges[j] looks along angle_min + j * increment; the last beam is the sensor's zero.
  EXPECT_NEAR(s.angle_min, s.angle_increment, 1e-6);
  EXPECT_NEAR(s.angle_increment, 2.0 * M_PI / 2160, 1e-6);
  EXPECT_FLOAT_EQ(s.ranges[2159], 1.000f);  // sensor angle 0 -> point 0
  // ROS angle +increment is the sensor's 360 - increment degrees -> the last point.
  EXPECT_FLOAT_EQ(s.ranges[0], 1.000f + 2159 * 0.001f);
  EXPECT_LT(s.time_increment, 0.0f);
  EXPECT_NEAR(s.time_increment, -1.0 / (kSpinHz * 2160), 1e-9);
}

TEST(ScanBinning, EveryBeamIsStampedAtItsMeasurementTime)
{
  for (bool ccw : {true, false}) {
    BinnedScan s;
    ASSERT_TRUE(ldlidar::bin_scan(revolution(2160), kSpinHz, params(2160, ccw), s));
    for (int j : {0, 1, 700, 1500, 2159}) {
      // The point in bin j was measured at kT0 + sensor_angle / spin; check it against the array.
      const int k = ccw ? 2159 - j : j;
      const double measured = kT0 + (360.0 * k / 2160) / (360.0 * kSpinHz);
      EXPECT_NEAR(s.stamp_s + j * s.time_increment, measured, 1e-6) << "ccw=" << ccw << " j=" << j;
    }
  }
}

TEST(ScanBinning, CwLayoutKeepsSensorOrder)
{
  BinnedScan s;
  ASSERT_TRUE(ldlidar::bin_scan(revolution(2160), kSpinHz, params(2160, false), s));
  EXPECT_FLOAT_EQ(s.angle_min, 0.0f);
  EXPECT_GT(s.time_increment, 0.0f);
  EXPECT_FLOAT_EQ(s.ranges[0], 1.000f);
  EXPECT_FLOAT_EQ(s.ranges[10], 1.010f);
  EXPECT_NEAR(s.stamp_s, kT0, 1e-6);
}

TEST(ScanBinning, BunchedArrivalTimesDoNotMoveTheStamp)
{
  // Packets read in bursts: arrival stamps scatter by +-3 ms around the true time.
  BinnedScan clean, bunched;
  ASSERT_TRUE(ldlidar::bin_scan(revolution(2160), kSpinHz, params(2160, true), clean));
  ASSERT_TRUE(ldlidar::bin_scan(revolution(2160, 0.003), kSpinHz, params(2160, true), bunched));
  EXPECT_NEAR(bunched.stamp_s, clean.stamp_s, 0.5e-3);
}

TEST(ScanBinning, LatencyIsSubtracted)
{
  BinnedScan a, b;
  auto p = params(2160, true);
  ASSERT_TRUE(ldlidar::bin_scan(revolution(2160), kSpinHz, p, a));
  p.stamp_latency_s = 0.004;
  ASSERT_TRUE(ldlidar::bin_scan(revolution(2160), kSpinHz, p, b));
  EXPECT_NEAR(a.stamp_s - b.stamp_s, 0.004, 1e-9);
}

TEST(ScanBinning, FewerPointsThanBinsLeavesNanAndMoreKeepsTheNearest)
{
  BinnedScan s;
  ASSERT_TRUE(ldlidar::bin_scan(revolution(1000), kSpinHz, params(2160, false), s));
  int nan = 0;
  for (float r : s.ranges) {
    nan += std::isnan(r);
  }
  EXPECT_NEAR(nan, 2160 - 1000, 2);

  auto pts = revolution(2160);
  pts.push_back({0.0f, 500, 90, pts[0].stamp_ns});  // a nearer return in bin 0
  ASSERT_TRUE(ldlidar::bin_scan(pts, kSpinHz, params(2160, false), s));
  EXPECT_FLOAT_EQ(s.ranges[0], 0.5f);
  EXPECT_FLOAT_EQ(s.intensities[0], 90.0f);
}

TEST(ScanBinning, InvalidAndOutOfRangePointsAreBlank)
{
  std::vector<RawPoint> pts = revolution(2160);
  pts[5].distance_mm = 0;
  pts[5].intensity = 0;          // the SDK filter's "removed" marker
  pts[6].distance_mm = 30000;    // beyond range_max
  pts[7].distance_mm = 10;       // inside range_min
  BinnedScan s;
  auto p = params(2160, false);
  p.range_max = 25.0f;
  ASSERT_TRUE(ldlidar::bin_scan(pts, kSpinHz, p, s));
  EXPECT_TRUE(std::isnan(s.ranges[5]));
  EXPECT_TRUE(std::isnan(s.ranges[6]));
  EXPECT_TRUE(std::isnan(s.ranges[7]));
  EXPECT_FALSE(std::isnan(s.ranges[8]));
}

TEST(ScanBinning, AngleCropBlanksTheSensorSector)
{
  BinnedScan s;
  auto p = params(2160, false);
  p.crop = true;
  p.crop_min_deg = 90.0;
  p.crop_max_deg = 180.0;
  ASSERT_TRUE(ldlidar::bin_scan(revolution(2160), kSpinHz, p, s));
  EXPECT_TRUE(std::isnan(s.ranges[600]));   // 100 degrees
  EXPECT_FALSE(std::isnan(s.ranges[300]));  // 50 degrees
}

TEST(ScanBinning, RefusesWhatCannotBeStamped)
{
  BinnedScan s;
  auto pts = revolution(2160);
  EXPECT_FALSE(ldlidar::bin_scan(pts, 0.0, params(2160, true), s));
  for (auto & p : pts) {
    p.stamp_ns = 0;
  }
  EXPECT_FALSE(ldlidar::bin_scan(pts, kSpinHz, params(2160, true), s));
}
