// Standalone host for the lifecycle component, for launching it as an ordinary (respawnable)
// process under a nav2 lifecycle_manager instead of inside a component container.

#include <cstdio>
#include <cstdlib>
#include <memory>

#include "ldlidar_component.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ldlidar::LdLidarComponent>();
  rclcpp::spin(node->get_node_base_interface());
  // A driver that gave up on a silent sensor exits with an error, for its supervisor to restart
  // (the port is reopened by the new process).
  const int status = node->failed() ? 1 : 0;
  rclcpp::shutdown();
  if (status != 0) {
    // Leave without tearing the driver down. The SDK's serial Close() closes the port and sets
    // its handle to -1 while its own read thread is still inside ReadFromIO(), and glibc aborts
    // FD_SET(-1, ...) ("bit out of range 0 - FD_SETSIZE"): returning from main destroyed the
    // driver and the supervisor saw SIGABRT (and a core dump) in place of this status. The
    // operating system closes the port.
    std::fflush(stdout);
    std::fflush(stderr);
    std::_Exit(status);
  }
  return status;
}
