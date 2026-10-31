/**
 * @file data_watchdog.hpp
 * @brief How long the sensor's data has been missing, and when that is a failure.
 *
 * No ROS types, so it is unit-tested without a node (test/test_data_watchdog.cpp).
 *
 * The SDK reports a time-out when no revolution has arrived for a while, and the lidar thread
 * used to log it and carry on for ever: a USB-serial link that stalled (a cable knocked, the
 * adapter reset) left a driver that looked alive, with diagnostics reading OK, that no
 * respawn would ever restart. This says "failed" once the data has been missing for
 * `fail_after_s`, so the node can report ERROR and exit for its supervisor to start it again
 * (which reopens the port).
 */

#ifndef DATA_WATCHDOG_HPP_
#define DATA_WATCHDOG_HPP_

namespace ldlidar
{

class DataWatchdog
{
public:
  enum class State
  {
    kOk,        ///< data is arriving
    kMissing,   ///< a time-out, not yet long enough to give up
    kFailed,    ///< missing for `fail_after_s`
  };

  /// `fail_after_s` of zero or less never fails (the state still says "missing").
  explicit DataWatchdog(double fail_after_s)
  : fail_after_s_(fail_after_s) {}

  /// One pass of the lidar thread at `now_s` (any monotonic clock): `timed_out` is the SDK's
  /// verdict for this pass, `got_scan` whether a revolution was just read. A pass that is
  /// neither (the SDK is waiting for the next revolution) changes nothing.
  State update(bool timed_out, bool got_scan, double now_s)
  {
    if (got_scan) {
      missing_ = false;
    } else if (timed_out && !missing_) {
      missing_ = true;
      since_s_ = now_s;
    }
    if (!missing_) {
      return State::kOk;
    }
    if (fail_after_s_ > 0.0 && now_s - since_s_ >= fail_after_s_) {
      return State::kFailed;
    }
    return State::kMissing;
  }

  bool missing() const {return missing_;}

private:
  double fail_after_s_;
  bool missing_{false};
  double since_s_{0.0};
};

}  // namespace ldlidar

#endif  // DATA_WATCHDOG_HPP_
