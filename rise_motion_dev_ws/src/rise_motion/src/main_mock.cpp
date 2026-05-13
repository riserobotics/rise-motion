#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rise_motion/ec_manager_mock.hpp>
#include <rise_motion/ethercat_node.hpp>

int main(int argc, char *argv[]) {
  // num_motors mirrors real EtherCAT slave discovery: determined at startup, not runtime.
  // Pass as first non-ROS argument: ros2 run rise_motion rise_motion_mock -- 2
  int num_motors = 6;
  for (int i = 1; i < argc; ++i) {
    if (argv[i][0] != '-' && std::isdigit(static_cast<unsigned char>(argv[i][0]))) {
      num_motors = std::atoi(argv[i]);
      break;
    }
  }

  rclcpp::init(argc, argv);

  MockECManager ec_manager(/*cycle_period_ms=*/1, num_motors);
  auto node = std::make_shared<EthercatNode>(ec_manager);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
