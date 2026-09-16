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

  // Costmap2DROS uses the supplied name both as its namespace and lifecycle
  // node name. This intentionally produces /static_costmap/static_costmap and
  // publishes the public OccupancyGrid at /static_costmap/costmap.
  auto costmap =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("static_costmap");
  rclcpp::spin(costmap->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
