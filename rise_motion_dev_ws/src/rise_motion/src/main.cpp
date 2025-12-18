#include <chrono>
#include <rise_motion/ec_manager.hpp>
#include <rise_motion/ec_structs.hpp>
#include <rise_motion/snapshot.hpp>
#include <thread>

int main(int argc, char *argv[]) {
  (void)argc;
  (void)argv;

  Snapshot<MotorInputs> inputs;
  Snapshot<MotorOutputs> outputs;
  ECManager ec_manager("eno1", inputs, outputs);
  ec_manager.init_ec();
  std::thread ec_manager_thread(&ECManager::cyclic_loop, &ec_manager);

  MotorInputs local_inputs;
  MotorOutputs local_outputs;
  auto next = std::chrono::steady_clock::now();
  auto period = std::chrono::milliseconds(1);
  long long counter = 0;
  for (;;) {
    next += period;
    counter++;
    inputs.write(local_inputs);
    outputs.read(local_outputs);
    if (counter % 1000 == 0) {
      print_motor_inputs(&local_inputs);
      print_motor_outputs(&local_outputs);
    }
    std::this_thread::sleep_until(next);
  }
  ec_manager_thread.join();
  return 0;
}
