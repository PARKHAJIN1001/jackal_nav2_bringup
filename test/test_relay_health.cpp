// Copyright 2026 parkhajin
#include "gtest/gtest.h"
#include "jackal_nav2_bringup/relay_health.hpp"

using jackal_nav2_bringup::RelayHealth;

TEST(RelayHealth, SeparatesMeasurementAgeFromReceipt)
{
  RelayHealth h;
  EXPECT_EQ(h.reason(100, 10), "waiting_for_cloud");
  h.observe(99, 100, 10, true);
  EXPECT_EQ(h.reason(100.01, 10.01), "measurement_stale");
  h.observe(100.1, 100.15, 10.1, true);
  EXPECT_EQ(h.reason(100.2, 10.2), "fresh");
  // Paused ROS time cannot hide a real sensor receipt timeout.
  EXPECT_EQ(h.reason(100.2, 10.5), "receipt_timeout");
}

TEST(RelayHealth, FutureRegressionInvalidAndRecovery)
{
  RelayHealth h;
  h.observe(101, 100, 10, true);
  EXPECT_EQ(h.reason(100, 10), "measurement_in_future");
  h.observe(100, 100.01, 10.01, true);
  EXPECT_EQ(h.reason(100.01, 10.01), "stamp_regression");
  EXPECT_EQ(h.stamp_regressions, 1U);
  h.observe(100.02, 100.03, 10.02, true);
  EXPECT_EQ(h.reason(100.03, 10.02), "fresh");
  EXPECT_EQ(h.stamp_regressions, 1U);
  h.observe(0, 100.04, 10.03, false);
  EXPECT_EQ(h.reason(100.04, 10.03), "invalid_header");
  h.observe(90, 90.1, 10.04, true);
  EXPECT_EQ(h.reason(90.1, 10.04), "clock_regression");
  EXPECT_EQ(h.clock_regressions, 1U);
  EXPECT_EQ(h.count, 5U);
}

TEST(RelayHealth, KeepsOnlyCountersNotGrowingHistory)
{
  RelayHealth h;
  for (int i = 0; i < 100000; ++i) {
    h.observe(100 + i * 0.1, 100.01 + i * 0.1, 10 + i * 0.1, true);
  }
  EXPECT_EQ(h.count, 100000U);
  EXPECT_NEAR(h.interval, 0.1, 1e-8);
  EXPECT_NEAR(h.max_interval, 0.1, 1e-8);
  EXPECT_EQ(h.stamp_regressions, 0U);
}
