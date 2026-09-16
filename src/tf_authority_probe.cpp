// Copyright 2026 parkhajin
// Read-only TF authority collection. Humble rclpy omits publisher GIDs.
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

using Json = nlohmann::json;

template<typename Gid>
std::string hex_gid(const Gid & data)
{
  std::ostringstream output;
  for (size_t i = 0; i < RMW_GID_STORAGE_SIZE; ++i) {
    output << std::hex << std::setw(2) << std::setfill('0') << int(data[i]);
  }
  return output.str();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("tf_authority_probe");
  const double duration = node->declare_parameter("duration", 12.0);
  const double warmup = node->declare_parameter("warmup", 3.0);
  const auto start = std::chrono::steady_clock::now();
  if (!std::isfinite(duration) || duration < 3.0) {
    std::cerr << "duration must be finite and at least 3 seconds\n";
    rclcpp::shutdown();
    return 2;
  }
  using Key = std::tuple<std::string, std::string, std::string>;
  std::map<Key, Json> edges;
  std::map<std::pair<Key, std::string>, double> previous;
  std::map<std::pair<Key, std::string>, double> previous_receipt;
  std::map<std::pair<Key, std::string>, double> restart_until;
  std::vector<rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr> subscriptions;
  const std::vector<std::string> topics = {
    "/tf", "/tf_static", "/j100_0519/tf", "/j100_0519/tf_static"};
  for (const auto & topic : topics) {
    auto qos = rclcpp::QoS(100);
    if (topic.find("tf_static") != std::string::npos) {
      qos.transient_local();
    } else {
      qos.best_effort();
    }
    subscriptions.push_back(
      node->create_subscription<tf2_msgs::msg::TFMessage>(
        topic, qos,
        [&, topic](tf2_msgs::msg::TFMessage::ConstSharedPtr msg,
        const rclcpp::MessageInfo & info) {
          const auto writer = hex_gid(info.get_rmw_message_info().publisher_gid.data);
          for (const auto & tf : msg->transforms) {
            Key key{topic, tf.header.frame_id, tf.child_frame_id};
            const double stamp = rclcpp::Time(tf.header.stamp).seconds();
            const double receive_ros = node->now().seconds();
            const double elapsed = std::chrono::duration<double>(
              std::chrono::steady_clock::now() - start).count();
            const double age = receive_ros - stamp;
            auto found = edges.find(key);
            if (found == edges.end()) {
              edges[key] = {
                {"topic", topic}, {"parent", tf.header.frame_id},
                {"child", tf.child_frame_id}, {"count", 0},
                {"age_min_sec", age}, {"age_max_sec", age},
                {"stamp_regressions", 0}, {"invalid_quaternion", 0},
                {"regression_events", Json::array()}, {"phases", Json::object()},
                {"max_stamp_regression_sec", 0.0},
                {"writers", Json::object()}};
            }
            auto & edge = edges[key];
            edge["latest_stamp_sec"] = stamp;
            edge["writers"][writer] = "unknown";
            edge["count"] = edge["count"].get<int>() + 1;
            edge["age_min_sec"] = std::min(age, edge["age_min_sec"].get<double>());
            edge["age_max_sec"] = std::max(age, edge["age_max_sec"].get<double>());
            auto old = previous.find({key, writer});
            const bool backwards = old != previous.end() && stamp < old->second;
            const bool restart_candidate = topic == "/tf" && old != previous.end() &&
            ((backwards && old->second - stamp > 0.5) ||
            elapsed - previous_receipt[{key, writer}] > 2.0);
            if (restart_candidate) {restart_until[{key, writer}] = elapsed + warmup;}
            const std::string phase = elapsed < warmup ? "initialization" :
            (elapsed < restart_until[{key, writer}] ? "restart_candidate" : "steady");
            if (!edge["phases"].contains(phase)) {
              edge["phases"][phase] = {{"count", 0}, {"stamp_regressions", 0}};
            }
            auto & phase_stats = edge["phases"][phase];
            phase_stats["count"] = phase_stats["count"].get<int>() + 1;
            if (old != previous.end() && stamp < old->second) {
              edge["stamp_regressions"] = edge["stamp_regressions"].get<int>() + 1;
              phase_stats["stamp_regressions"] = phase_stats["stamp_regressions"].get<int>() + 1;
              const double delta = old->second - stamp;
              edge["max_stamp_regression_sec"] = std::max(
                delta, edge["max_stamp_regression_sec"].get<double>());
              if (edge["regression_events"].size() < 64) {
                edge["regression_events"].push_back(
            {
              {"previous_stamp_sec", old->second}, {"current_stamp_sec", stamp},
              {"regression_sec", delta}, {"receive_ros_sec", receive_ros},
              {"receive_elapsed_sec", elapsed}, {"writer", writer}, {"phase", phase}});
              }
            }
            previous[{key, writer}] = stamp;
            previous_receipt[{key, writer}] = elapsed;
            const auto & q = tf.transform.rotation;
            const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3) {
              edge["invalid_quaternion"] = edge["invalid_quaternion"].get<int>() + 1;
            }
          }
        }));
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count() < duration)
  {
    executor.spin_once(std::chrono::milliseconds(20));
  }
  Json result = Json::array();
  for (auto & entry : edges) {
    auto & edge = entry.second;
    for (const auto & endpoint : node->get_publishers_info_by_topic(std::get<0>(entry.first))) {
      auto gid = hex_gid(endpoint.endpoint_gid());
      if (edge["writers"].contains(gid)) {
        auto ns = endpoint.node_namespace();
        edge["writers"][gid] = (ns == "/" ? "/" : ns + "/") + endpoint.node_name();
      }
    }
    std::vector<std::string> writers;
    std::set<std::string> owners;
    for (auto it = edge["writers"].begin(); it != edge["writers"].end(); ++it) {
      writers.push_back(it.key());
      owners.insert(it.value().get<std::string>());
    }
    edge["owners_by_writer"] = edge["writers"];
    edge["writers"] = writers;
    edge["owners"] = owners;
    edge["observed_rate_hz"] = edge["count"].get<double>() / duration;
    edge["latest_age_sec"] = node->now().seconds() - edge["latest_stamp_sec"].get<double>();
    result.push_back(edge);
  }
  std::cout << result.dump() << std::endl;
  rclcpp::shutdown();
  return 0;
}
