// Copyright 2026 parkhajin
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "jackal_nav2_bringup/relay_health.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace jackal_nav2_bringup
{

class PointCloudRelay : public rclcpp::Node
{
public:
  PointCloudRelay()
  : Node("pointcloud_relay")
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/livox/lidar");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/livox/lidar_local");
    const auto input_depth = declare_parameter<int>("input_depth", 10);
    const auto output_depth = declare_parameter<int>("output_depth", 10);

    if (input_topic.empty() || output_topic.empty()) {
      throw std::invalid_argument("input_topic and output_topic must not be empty");
    }
    if (input_topic == output_topic) {
      throw std::invalid_argument("input_topic and output_topic must be different");
    }
    if (input_depth <= 0 || output_depth <= 0) {
      throw std::invalid_argument("input_depth and output_depth must be positive");
    }

    // The Livox publisher offers RELIABLE data, but the current Fast DDS robot
    // link only delivers this stream consistently to SensorDataQoS readers.
    // Bound this relay's DDS histories, not those of downstream consumers.
    // BEST_EFFORT readers can also connect directly to a RELIABLE writer.
    const auto input_qos = rclcpp::QoS(rclcpp::KeepLast(input_depth))
      .best_effort()
      .durability_volatile();
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(output_depth))
      .best_effort()
      .durability_volatile();

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, output_qos);
    if (get_node_topics_interface()->resolve_topic_name(input_topic) ==
      get_node_topics_interface()->resolve_topic_name(output_topic))
    {
      throw std::invalid_argument("input/output resolve to the same topic (relay loop)");
    }
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/nav2/lidar_relay_diagnostics", 1);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic,
      input_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        health_.observe(
          message->header.stamp.sec + message->header.stamp.nanosec * 1e-9,
          now().seconds(), steady_seconds(),
          !message->header.frame_id.empty() && message->header.stamp.nanosec < 1000000000U);
        // Preserve measurement time, coordinates and points, including people.
        // Diagnostics do not silently drop/re-date data or replace the safety Guard.
        publisher_->publish(*message);
      });

    health_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() {
        const auto current = now();
        const double mono = steady_seconds();
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "pointcloud_relay";
        status.hardware_id = "mid360_network_input";
        status.message = health_.reason(current.seconds(), mono);
        status.level = status.message == "fresh" ? status.OK : status.WARN;
        const auto value = [&status](const std::string & key, double number) {
          diagnostic_msgs::msg::KeyValue item;
          item.key = key;
          item.value = std::to_string(number);
          status.values.push_back(item);
        };
        value("received_count", health_.count);
        if (health_.count > 0) {
          value("measurement_stamp_sec", health_.stamp);
          value("measurement_age_sec", current.seconds() - health_.stamp);
          value("age_at_receipt_sec", health_.age_at_receipt);
          value("receipt_silence_sec", mono - health_.receipt);
          value("last_interval_sec", health_.interval);
          value("max_interval_sec", health_.max_interval);
        }
        value("stamp_regressions", health_.stamp_regressions);
        value("clock_regressions", health_.clock_regressions);
        diagnostic_msgs::msg::DiagnosticArray report;
        report.header.stamp = current;
        report.status.push_back(status);
        diagnostics_->publish(report);
        if (status.level != status.OK) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "PointCloud2 relay health: %s (see /nav2/lidar_relay_diagnostics)",
            status.message.c_str());
        }
      });

    RCLCPP_INFO(
      get_logger(),
      "Relaying PointCloud2 from %s to bounded local topic %s",
      input_topic.c_str(), output_topic.c_str());
  }

private:
  static double steady_seconds()
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  RelayHealth health_;
};

}  // namespace jackal_nav2_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<jackal_nav2_bringup::PointCloudRelay>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("pointcloud_relay"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
