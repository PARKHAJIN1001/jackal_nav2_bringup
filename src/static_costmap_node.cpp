// Copyright 2026 parkhajin
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // The NodeOptions constructor makes this an independent lifecycle node.
  // The name-based constructor is a follower: it skips preshutdown cleanup
  // and expects a parent to deactivate and join its costmap update thread.
  // Preserve the public namespace and YAML parameter selection explicitly.
  auto options = rclcpp::NodeOptions().arguments(
  {
    "--ros-args", "-r", "__node:=static_costmap", "-r", "__ns:=/static_costmap"});
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
  rclcpp::spin(costmap->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
