// Copyright 2026 parkhajin
#ifndef JACKAL_NAV2_BRINGUP__RELAY_HEALTH_HPP_
#define JACKAL_NAV2_BRINGUP__RELAY_HEALTH_HPP_

#include <cmath>
#include <cstddef>
#include <limits>
#include <string>

namespace jackal_nav2_bringup
{

// Fixed-size metadata only. The relay never retains a PointCloud2 history here.
struct RelayHealth
{
  std::size_t count{0}, stamp_regressions{0}, clock_regressions{0};
  double stamp{0}, receipt{0}, age_at_receipt{0}, interval{0}, max_interval{0};
  double last_ros{std::numeric_limits<double>::quiet_NaN()};
  bool valid_header{false}, last_stamp_regressed{false}, last_clock_regressed{false};

  void observe(double measurement, double ros_now, double mono, bool header_valid)
  {
    last_clock_regressed = std::isfinite(last_ros) && ros_now < last_ros;
    if (last_clock_regressed) {++clock_regressions;}
    last_ros = ros_now;
    last_stamp_regressed = count > 0 && header_valid && valid_header && measurement < stamp;
    if (last_stamp_regressed) {++stamp_regressions;}
    if (count > 0) {
      interval = mono - receipt;
      if (interval > max_interval) {max_interval = interval;}
    }
    ++count;
    stamp = measurement;
    receipt = mono;
    age_at_receipt = ros_now - measurement;
    valid_header = header_valid && std::isfinite(measurement) && measurement > 0;
  }

  std::string reason(double ros_now, double mono) const
  {
    if (count == 0) {return "waiting_for_cloud";}
    if (last_clock_regressed || ros_now < last_ros) {return "clock_regression";}
    if (!valid_header) {return "invalid_header";}
    if (mono - receipt > 0.30) {return "receipt_timeout";}
    if (ros_now - stamp > 0.30) {return "measurement_stale";}
    if (stamp - ros_now > 0.05) {return "measurement_in_future";}
    if (last_stamp_regressed) {return "stamp_regression";}
    return "fresh";
  }
};

}  // namespace jackal_nav2_bringup
#endif  // JACKAL_NAV2_BRINGUP__RELAY_HEALTH_HPP_
