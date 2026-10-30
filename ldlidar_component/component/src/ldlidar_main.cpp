// Standalone host for the lifecycle component, for launching it as an ordinary (respawnable)
// process under a nav2 lifecycle_manager instead of inside a component container.

#include <memory>

#include "ldlidar_component.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ldlidar::LdLidarComponent>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
