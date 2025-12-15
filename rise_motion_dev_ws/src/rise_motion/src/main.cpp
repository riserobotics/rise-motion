#include <chrono>
#include <cstdint>
#include <rise_motion/ec_manager.hpp>
#include <thread>
#include <vector>

int main(int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  std::vector<uint32_t> desired_motor_values;
  std::vector<uint32_t> actual_motor_values;
  desired_motor_values.resize(6);
  actual_motor_values.resize(6);

  ECManager ec_manager("eno1");
  ec_manager.init_ec();

  std::thread ec_manager_thread(&ECManager::cyclic_loop, &ec_manager);

  for (int pos = 0; pos <= 10000; pos++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ec_manager.set_motor_values(desired_motor_values);
    ec_manager.get_motor_values(actual_motor_values);
  }
  ec_manager_thread.join();
  return 0;
}
