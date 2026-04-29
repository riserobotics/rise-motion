#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rise_motion/ec_manager_mock.hpp>
#include <rise_motion/ethercat_node.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  MockECManager ec_manager(/*cycle_period_ms=*/1, /*num_motors=*/6);
  auto node = std::make_shared<EthercatNode>(ec_manager);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
