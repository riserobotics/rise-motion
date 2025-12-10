#include <rclcpp/rclcpp.hpp>
#include <cstring>
#include <cstdio>

extern "C" {
#include "soem/soem.h"  // SOEM library header
}

#include "rise_motion/msg/motor_command_array.hpp"
#include "rise_motion/msg/motor_feedback_array.hpp"


// Simple structure representing data we send to a motor (R-PDO - Process Data Object)
struct MotorOutput {
  uint16_t Controlword;
  int8_t OpMode;
  int16_t TargetTorque;
  int32_t TargetPosition;
  int32_t TargetVelocity;
  int16_t TorqueOffset;
  int32_t TuningCommand;
  int32_t PhysicalOutputs;
  int32_t BitMask;
  int32_t UserMOSI;
  int32_t VelocityOffset;
} __attribute__((packed)); // Ensure no padding between fields

// Simple structure representing data we receive from a motor (T-PDO)
struct MotorInput {
  uint16_t Statusword;
  int8_t OpModeDisplay;
  int32_t PositionValue;
  int32_t VelocityValue;
  int16_t TorqueValue;
  uint16_t AnalogInput1;
  uint16_t AnalogInput2;
  uint16_t AnalogInput3;
  uint16_t AnalogInput4;
  uint32_t TuningStatus;
  uint32_t DigitalInputs;
  uint32_t UserMISO;
  uint32_t Timestamp;
  int32_t PositionDemandInternalValue;
  int32_t VelocityDemandValue;
  int16_t TorqueDemand;
} __attribute__((packed));

class EthercatInitializer : public rclcpp::Node
{
public:
    EthercatInitializer() : Node("ethercat_initializer")
    {
        RCLCPP_INFO(this->get_logger(), "EtherCAT Initializer Starting");

        // Get parameters
        this->declare_parameter<std::string>("ethercat_interface", "eth0");

        interface_ = this->get_parameter("ethercat_interface").as_string();

        RCLCPP_INFO(this->get_logger(), "Interface: %s", interface_.c_str());

        // Create ROS subscribers/publishers
        command_sub_ = this->create_subscription<rise_motion::msg::MotorCommandArray>(
            "SOEM_input_topic", 10,
            [this](const rise_motion::msg::MotorCommandArray::SharedPtr msg) {
                // Just update our target positions
                for (size_t i = 0; i < std::min(msg->target_positions.size(), target_positions_.size()); i++) {
                    target_positions_[i] = msg->target_positions[i];
                    RCLCPP_INFO(this->get_logger(), "Updated motor %zu target: %d", i, target_positions_[i]);
                }
            });

        feedback_pub_ = this->create_publisher<rise_motion::msg::MotorFeedbackArray>(
            "motor_feedback", 10);

        // Initialize EtherCAT
        if (!init_ethercat()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize EtherCAT");
            return;
        }

        // Create timer for cyclic communication (1 kHz = 1ms period)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&EthercatInitializer::cyclic_loop, this));
    }

    ~EthercatInitializer()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down EtherCAT");
        if (initialized_) {
            // Request INIT state for all slaves
            context_.slavelist[0].state = EC_STATE_INIT;
            ecx_writestate(&context_, 0);

            // Close socket
            ecx_close(&context_);
        }
    }

private:
    bool init_ethercat()
    {
        RCLCPP_INFO(this->get_logger(), "\n=== STEP 1: Initialize SOEM Context ===");

        // Zero-fill context to avoid surprises
        memset(&context_, 0, sizeof(context_));

        // Initialize SOEM on the specified network interface / opens a raw socket for EtherCAT communication
        if (!ecx_init(&context_, interface_.c_str())) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize socket on %s", interface_.c_str());
            RCLCPP_ERROR(this->get_logger(), "Make sure to run with sudo and interface exists!");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "✓ Socket initialized on %s", interface_.c_str());

        RCLCPP_INFO(this->get_logger(), "\n=== STEP 2: Find EtherCAT Slaves ===");

        // Scan the EtherCAT network for slaves (motors/drives)
        // SOEM automatically enumerates all devices on the network
        if (ecx_config_init(&context_) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "No slaves found!");
            ecx_close(&context_);
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "✓ Found %d slaves on the network", context_.slavecount);

        // Set number of motors based on discovered slaves
        num_motors_ = context_.slavecount;

        // Initialize target positions and fault state tracking based on discovered motors
        target_positions_.resize(num_motors_, 0);
        motor_in_fault_.resize(num_motors_, false);
        fault_reset_sent_.resize(num_motors_, false);

        RCLCPP_INFO(this->get_logger(), "Configured for %d motors (auto-detected)", num_motors_);

        // Print information about each slave
        for (int i = 1; i <= context_.slavecount; i++) {
            ec_slavet* slave = &context_.slavelist[i];
            RCLCPP_INFO(this->get_logger(), "  Slave %d: %s", i, slave->name);
            RCLCPP_INFO(this->get_logger(), "    Vendor ID: 0x%08X", slave->eep_man);
            RCLCPP_INFO(this->get_logger(), "    Product Code: 0x%08X", slave->eep_id);
        }

        RCLCPP_INFO(this->get_logger(), "\n=== STEP 3: Map Process Data (IO Map) ===");

        // Map the process data
        // SOEM creates a shared memory buffer (io_map_) where:
        // - We write output data (commands to motors)
        // - We read input data (feedback from motors)
        // Each slave gets a section of this buffer
        ecx_config_map_group(&context_, io_map_, 0);

        RCLCPP_INFO(this->get_logger(), "✓ Process data mapped");

        // Show the memory layout for each slave
        for (int i = 1; i <= context_.slavecount; i++) {
            ec_slavet* slave = &context_.slavelist[i];
            RCLCPP_INFO(this->get_logger(), "  Slave %d IO mapping:", i);
            RCLCPP_INFO(this->get_logger(), "    Outputs (Master->Slave): %d bytes", slave->Obytes);
            RCLCPP_INFO(this->get_logger(), "    Inputs (Slave->Master): %d bytes", slave->Ibytes);
        }

        RCLCPP_INFO(this->get_logger(), "\n=== STEP 4: Transition to OPERATIONAL State ===");

        // Wait for slaves to reach SAFE_OP state
        ecx_statecheck(&context_, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

        // Send one valid process data frame
        ecx_send_processdata(&context_);
        ecx_receive_processdata(&context_, EC_TIMEOUTRET);

        // Request OPERATIONAL state
        context_.slavelist[0].state = EC_STATE_OPERATIONAL;
        ecx_writestate(&context_, 0);

        // Wait for OPERATIONAL state
        int wait_count = 40;
        do {
            ecx_send_processdata(&context_);
            ecx_receive_processdata(&context_, EC_TIMEOUTRET);
            ecx_statecheck(&context_, 0, EC_STATE_OPERATIONAL, 50000);
        } while (wait_count-- && (context_.slavelist[0].state != EC_STATE_OPERATIONAL));

        if (context_.slavelist[0].state == EC_STATE_OPERATIONAL) {
            RCLCPP_INFO(this->get_logger(), "✓ All slaves in OPERATIONAL state");
            initialized_ = true;
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "✗ Failed to reach OPERATIONAL state");
            ecx_close(&context_);
            return false;
        }
    }

    void cyclic_loop()
    {
        if (!initialized_) return;

        // STEP 1: WRITE OUTPUT DATA (Commands to motors)
        for (int i = 1; i <= context_.slavecount; i++) {
            ec_slavet* slave = &context_.slavelist[i];

            // Get pointer to this slave's output section in the IO map
            uint8_t* output_ptr = slave->outputs;

            if (output_ptr == nullptr || slave->Obytes < 6) {
                continue;
            }

            // Build our output data structure - zero-initialize all fields
            MotorOutput output = {};

            int motor_idx = i - 1;

            // Check if motor is in fault state and needs reset
            if (motor_in_fault_[motor_idx] && !fault_reset_sent_[motor_idx]) {
                // Send fault reset command: bit 7 of controlword = 1
                output.Controlword = 0x0080;  // Fault reset (bit 7 = 1)
                fault_reset_sent_[motor_idx] = true;
                RCLCPP_WARN(this->get_logger(), "Motor %d: FAULT detected! Sending fault reset command (0x0080)", motor_idx);
            } else if (motor_in_fault_[motor_idx] && fault_reset_sent_[motor_idx]) {
                // Fault reset already sent, stop sending commands
                output.Controlword = 0x0000;  // Stop sending commands
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Motor %d: Fault reset sent, waiting for recovery", motor_idx);
            } else {
                // Normal operation
                output.Controlword = 0x000F;  // Enable operation (simplified)
                output.TargetPosition = target_positions_[motor_idx];
            }

            // Write to the IO map
            // THIS IS WHERE WE "BUILD THE FRAME" - we write bytes to memory
            memcpy(output_ptr, &output, sizeof(MotorOutput));

            // All MotorOutput fields are written to the IO map
            // SOEM will take this data and wrap it in an EtherCAT frame
        }

        // STEP 2: SEND THE ETHERCAT FRAME
        // SOEM takes all data from io_map_ and:
        // 1. Wraps it in EtherCAT protocol headers
        // 2. Wraps that in an Ethernet frame
        // 3. Sends it via raw socket to all slaves
        ecx_send_processdata(&context_);

        // STEP 3: RECEIVE THE RESPONSE FRAME
        // SOEM:
        // 1. Waits for response from slaves
        // 2. Receives the Ethernet/EtherCAT frame
        // 3. Unpacks the data into io_map_
        int wkc = ecx_receive_processdata(&context_, EC_TIMEOUTRET);

        // Working counter tells us if communication was successful
        if (wkc < 3 * context_.slavecount) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Communication issue - working counter: %d", wkc);
        }

        // STEP 4: READ INPUT DATA (Feedback from motors)
        auto feedback_msg = rise_motion::msg::MotorFeedbackArray();

        for (int i = 1; i <= context_.slavecount; i++) {
            ec_slavet* slave = &context_.slavelist[i];

            // Get pointer to this slave's input section in the IO map
            uint8_t* input_ptr = slave->inputs;

            if (input_ptr == nullptr || slave->Ibytes < 6) {
                continue;
            }

            // Read from the IO map
            MotorInput input;
            memcpy(&input, input_ptr, sizeof(MotorInput));

            // All MotorInput fields are read from the IO map
            // SOEM already unpacked this from the received EtherCAT frame

            // Detect fault condition (bit 3 of status word)
            int motor_idx = i - 1;
            bool fault_bit = (input.Statusword & 0x0008) != 0;

            // Update fault state
            if (fault_bit && !motor_in_fault_[motor_idx]) {
                motor_in_fault_[motor_idx] = true;
                RCLCPP_ERROR(this->get_logger(), "Motor %d: FAULT STATE detected (Status: 0x%04X)", motor_idx, input.Statusword);
            } else if (!fault_bit && motor_in_fault_[motor_idx]) {
                // Fault cleared
                motor_in_fault_[motor_idx] = false;
                fault_reset_sent_[motor_idx] = false;
                RCLCPP_INFO(this->get_logger(), "Motor %d: Fault cleared, returning to normal operation", motor_idx);
            }

            feedback_msg.status_words.push_back(input.Statusword);
            feedback_msg.actual_positions.push_back(input.PositionValue);
            feedback_msg.velocity.push_back(input.VelocityValue);
        }

        // Publish feedback
        feedback_pub_->publish(feedback_msg);

        // Print status
        static int counter = 0;
        if (++counter >= 1000) {  // Every 1 second
            counter = 0;
            if (!feedback_msg.status_words.empty()) {
                print_motor_status(0, feedback_msg);
            }
        }
    }

    // Helper function to decode and print CiA 402 status word bits
    void print_motor_status(int motor_idx, const rise_motion::msg::MotorFeedbackArray& feedback)
    {
        if (motor_idx >= static_cast<int>(feedback.status_words.size())) {
            return;
        }

        uint16_t status = feedback.status_words[motor_idx];

        // CiA 402 Status Word bit definitions
        bool ready_to_switch_on = (status & 0x0001) != 0;  // Bit 0
        bool switched_on = (status & 0x0002) != 0;         // Bit 1
        bool operation_enabled = (status & 0x0004) != 0;   // Bit 2
        bool fault = (status & 0x0008) != 0;               // Bit 3
        bool voltage_enabled = (status & 0x0010) != 0;     // Bit 4
        bool quick_stop = (status & 0x0020) != 0;          // Bit 5
        bool switch_on_disabled = (status & 0x0040) != 0;  // Bit 6
        bool warning = (status & 0x0080) != 0;             // Bit 7
        bool manufacturer_specific = (status & 0x0100) != 0; // Bit 8
        bool remote = (status & 0x0200) != 0;              // Bit 9
        bool target_reached = (status & 0x0400) != 0;      // Bit 10
        bool internal_limit_active = (status & 0x0800) != 0; // Bit 11

        RCLCPP_INFO(this->get_logger(), "\n=== Motor %d Status ===", motor_idx);
        RCLCPP_INFO(this->get_logger(), "Statusword: 0x%04X", status);
        RCLCPP_INFO(this->get_logger(), "  Ready to Switch On:  %s", ready_to_switch_on ? "YES" : "NO");
        RCLCPP_INFO(this->get_logger(), "  Switched On:         %s", switched_on ? "YES" : "NO");
        RCLCPP_INFO(this->get_logger(), "  Operation Enabled:   %s", operation_enabled ? "YES" : "NO");
        RCLCPP_INFO(this->get_logger(), "  Fault:               %s", fault ? "YES" : "NO");
        RCLCPP_INFO(this->get_logger(), "  Voltage Enabled:     %s", voltage_enabled ? "YES" : "NO");
        RCLCPP_INFO(this->get_logger(), "  Quick Stop:          %s", quick_stop ? "ACTIVE" : "INACTIVE");
        RCLCPP_INFO(this->get_logger(), "  Switch On Disabled:  %s", switch_on_disabled ? "YES" : "NO");
        RCLCPP_INFO(this->get_logger(), "  Warning:             %s", warning ? "YES" : "NO");
        RCLCPP_INFO(this->get_logger(), "  Manufacturer Specific: %s", manufacturer_specific ? "YES" : "NO");
        RCLCPP_INFO(this->get_logger(), "  Remote:              %s", remote ? "YES" : "NO");
        RCLCPP_INFO(this->get_logger(), "  Target Reached:      %s", target_reached ? "YES" : "NO");
        RCLCPP_INFO(this->get_logger(), "  Internal Limit:      %s", internal_limit_active ? "ACTIVE" : "INACTIVE");

        RCLCPP_INFO(this->get_logger(), "OpMode Display: %d",
                    motor_idx < static_cast<int>(feedback.status_words.size()) ?
                    static_cast<int>(feedback.status_words[motor_idx]) : 0);
        RCLCPP_INFO(this->get_logger(), "Position: %d",
                    motor_idx < static_cast<int>(feedback.actual_positions.size()) ?
                    feedback.actual_positions[motor_idx] : 0);
        RCLCPP_INFO(this->get_logger(), "Velocity: %d",
                    motor_idx < static_cast<int>(feedback.velocity.size()) ?
                    feedback.velocity[motor_idx] : 0);
        RCLCPP_INFO(this->get_logger(), "Target Position: %d\n",
                    motor_idx < static_cast<int>(target_positions_.size()) ?
                    target_positions_[motor_idx] : 0);
    }

private:
    std::string interface_;
    int num_motors_;
    bool initialized_ = false;

    ecx_contextt context_;       // SOEM context - contains all EtherCAT state
    char io_map_[4096];          // SOEM's IO map - shared memory for process data
    std::vector<int32_t> target_positions_;

    // Fault state tracking
    std::vector<bool> motor_in_fault_;      // True if motor is currently in fault state
    std::vector<bool> fault_reset_sent_;    // True if fault reset command has been sent

    rclcpp::Subscription<rise_motion::msg::MotorCommandArray>::SharedPtr command_sub_;
    rclcpp::Publisher<rise_motion::msg::MotorFeedbackArray>::SharedPtr feedback_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EthercatInitializer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
