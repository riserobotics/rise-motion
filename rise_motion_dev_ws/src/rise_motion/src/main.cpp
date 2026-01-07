#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rise_motion/ec_manager.hpp>
#include <rise_motion/ethercat_node.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  ECManager ec_manager("eno1"); // TODO: Connect config from rise-os-core
  auto node = std::make_shared<EthercatNode>(ec_manager);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
