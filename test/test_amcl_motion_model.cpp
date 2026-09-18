// Copyright 2026 parkhajin
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_amcl/motion_model/motion_model.hpp"
#include "pluginlib/class_loader.hpp"
#include "yaml-cpp/yaml.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;

double angleDifference(double first, double second)
{
  return std::atan2(std::sin(first - second), std::cos(first - second));
}

struct Spread
{
  std::array<double, 3> mean;
  std::array<double, 3> rms;
};

class AmclMotionModel : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto params = YAML::LoadFile(JACKAL_NAV2_CONFIG_PATH)["amcl"]["ros__parameters"];
    model_ = loader_.createSharedInstance(params["robot_model_type"].as<std::string>());
    model_->initialize(
      params["alpha1"].as<double>(), params["alpha2"].as<double>(),
      params["alpha3"].as<double>(), params["alpha4"].as<double>(),
      params["alpha5"].as<double>());
    // Exercise the installed plugin without a ROS node or DDS participant.
    samples_.resize(20000);
    filter_.current_set = 0;
    filter_.sets[0].sample_count = static_cast<int>(samples_.size());
    filter_.sets[0].samples = samples_.data();
  }

  void reset(const pf_vector_t & pose)
  {
    for (auto & sample : samples_) {
      sample.pose = pose;
      sample.weight = 1.0 / samples_.size();
    }
    srand48(20260917);
  }

  Spread spread(const pf_vector_t & expected) const
  {
    Spread result{};
    for (const auto & sample : samples_) {
      const std::array<double, 3> error = {
        sample.pose.v[0] - expected.v[0],
        sample.pose.v[1] - expected.v[1],
        angleDifference(sample.pose.v[2], expected.v[2])};
      for (size_t i = 0; i < error.size(); ++i) {
        result.mean[i] += error[i] / samples_.size();
        result.rms[i] += error[i] * error[i] / samples_.size();
      }
    }
    for (auto & value : result.rms) {
      value = std::sqrt(value);
    }
    return result;
  }

  Spread rotateWithLateralShift(double lateral)
  {
    const pf_vector_t origin = {{0.0, 0.0, 0.0}};
    const pf_vector_t pose = {{0.0, lateral, 0.08}};
    reset(origin);
    model_->odometryUpdate(&filter_, pose, pose);
    return spread(pose);
  }

  // Keep the loader alive until its plugin instance has been destroyed.
  pluginlib::ClassLoader<nav2_amcl::MotionModel> loader_{
    "nav2_amcl", "nav2_amcl::MotionModel"};
  std::shared_ptr<nav2_amcl::MotionModel> model_;
  pf_t filter_{};
  std::vector<pf_sample_t> samples_;
};

TEST_F(AmclMotionModel, SmallLateralShiftDoesNotExplodeAtOneCentimetre)
{
  const auto below = rotateWithLateralShift(0.0099);
  const auto above = rotateWithLateralShift(0.0101);
  // The old model generated roughly metre / radian scale noise above 1 cm.
  // Use physical bounds and a continuity check, not a duplicate model formula.
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_LT(above.rms[i], 0.08);
    EXPECT_LT(above.rms[i], 1.5 * below.rms[i]);
    EXPECT_LT(std::abs(above.mean[i]), 0.005);
  }
}

TEST_F(AmclMotionModel, ZeroMotionDoesNotDiffuseParticles)
{
  const pf_vector_t pose = {{1.3, -2.0, 3.1}};
  const pf_vector_t delta = {{0.0, 0.0, 0.0}};
  reset(pose);
  model_->odometryUpdate(&filter_, pose, delta);
  for (const auto value : spread(pose).rms) {
    EXPECT_LT(value, 1e-12);
  }
}

TEST_F(AmclMotionModel, NoiseFreeForwardReverseAndLateralMotionPreservePose)
{
  model_->initialize(0.0, 0.0, 0.0, 0.0, 0.0);
  const std::array<pf_vector_t, 4> deltas = {{
    {{0.3, 0.0, 0.1}}, {{-0.3, 0.0, -0.1}},
    {{0.0, 0.02, 0.08}}, {{0.0, -0.02, -0.08}}}};
  for (const double heading : {0.0, 1.2, kPi - 0.02, -kPi + 0.02}) {
    for (const auto & local : deltas) {
      const pf_vector_t origin = {{2.0, -3.0, heading}};
      const pf_vector_t delta = {{
        std::cos(heading) * local.v[0] - std::sin(heading) * local.v[1],
        std::sin(heading) * local.v[0] + std::cos(heading) * local.v[1],
        local.v[2]}};
      const pf_vector_t pose = {{
        origin.v[0] + delta.v[0], origin.v[1] + delta.v[1],
        angleDifference(heading + delta.v[2], 0.0)}};
      reset(origin);
      model_->odometryUpdate(&filter_, pose, delta);
      for (const auto value : spread(pose).rms) {
        EXPECT_LT(value, 1e-12);
      }
    }
  }
}

TEST_F(AmclMotionModel, TurningWithLeverArmRemainsLocallyConcentrated)
{
  // A fixed 14 cm offset approximates the recorded in-place trajectory.
  // Its small lateral increments must not create a map-scale particle spread.
  constexpr double radius = 0.14;
  pf_vector_t previous = {{radius, 0.0, 0.0}};
  reset(previous);
  for (int step = 1; step <= 40; ++step) {
    const double yaw = step <= 20 ? 0.08 * step : 1.6 - 0.08 * (step - 20);
    const pf_vector_t pose = {{radius * std::cos(yaw), radius * std::sin(yaw), yaw}};
    const pf_vector_t delta = {{
      pose.v[0] - previous.v[0], pose.v[1] - previous.v[1],
      angleDifference(pose.v[2], previous.v[2])}};
    model_->odometryUpdate(&filter_, pose, delta);
    previous = pose;
  }
  const auto result = spread(previous);
  EXPECT_LT(std::hypot(result.rms[0], result.rms[1]), 0.5);
  EXPECT_LT(result.rms[2], 0.35);
}
}  // namespace
