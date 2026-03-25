#pragma once
#include <cstdint>
#include <vector>

class IECManager {
public:
  virtual ~IECManager() = default;

  virtual int  init_ec() = 0;
  virtual void cyclic_loop() = 0;
  virtual bool is_running() = 0;
  virtual void stop() = 0;

  virtual bool get_motor_values_apsa(std::vector<int32_t>& motor_values) = 0;
  virtual bool set_motor_values_apsa(const std::vector<int32_t>& motor_values) = 0;

  virtual bool sdo_read(uint16_t device_id, uint16_t index,
                        uint8_t subindex, std::vector<uint8_t>& value) = 0;
  virtual bool sdo_write(uint16_t device_id, uint16_t index,
                         uint8_t subindex, std::vector<uint8_t>& value) = 0;
};
