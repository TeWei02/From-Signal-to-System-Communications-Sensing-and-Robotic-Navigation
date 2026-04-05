/*
 * low_level_interface.cpp
 * ------------------------
 * Hardware abstraction layer for motor control, GPIO, and sensor I/O.
 *
 * Provides a single ROS2 node that manages:
 *   - Motor PWM / duty cycle output (GPIO + timer modules)
 *   - Wheel encoder input (interrupt-based counting)
 *   - IMU via I2C (SPI alternative support)
 *   - Emergency stop input
 *   - Battery voltage monitoring
 *
 * Publishes:
 *   /robot/wheel_ticks (std_msgs/Int32MultiArray) — left & right encoder counts
 *   /robot/imu/raw (sensor_msgs/Imu) — raw IMU measurements
 *   /robot/battery_state (sensor_msgs/BatteryState) — voltage & current
 *
 * Subscribes:
 *   /robot/motor_cmd (std_msgs/Float32MultiArray) — motor duty commands
 *
 * Notes:
 *   - Uses HAL (e.g., libperiph on Linux, hardware-specific on bare metal)
 *   - All timings and pin configs read from robot_params.yaml at startup
 *   - Watchdog timer for safety: resets motors if no command for >1 second
 */

#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

// TODO: Include HAL headers
// #include <periph_common.h>
// #include <driver_imu.h>

class LowLevelInterface : public rclcpp::Node {
public:
    LowLevelInterface() : Node("low_level_interface") {
        // TODO: Initialize GPIO, PWM, I2C, ADC from hardware description
        // TODO: Calibrate IMU (gyro bias, accel scale)
        // TODO: Start encoder/timer interrupt handlers
        
        auto sub_motor = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/robot/motor_cmd", 10,
            std::bind(&LowLevelInterface::motor_cmd_callback, this, std::placeholders::_1));
        
        pub_ticks = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/robot/wheel_ticks", 10);
        pub_imu = this->create_publisher<sensor_msgs::msg::Imu>(
            "/robot/imu/raw", 10);
        pub_battery = this->create_publisher<sensor_msgs::msg::BatteryState>(
            "/robot/battery_state", 10);
        
        RCLCPP_INFO(this->get_logger(), "LowLevelInterface: hardware init complete");
    }

private:
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_ticks;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery;
    
    void motor_cmd_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        // TODO: Convert FloatMultiArray (duty cycle [-1, 1]) to PWM register values
        // TODO: Write to GPIO/PWM peripheral
        // TODO: Log faults (e.g., over-current, stall)
        if (msg->data.size() >= 2) {
            double duty_left = msg->data[0];
            double duty_right = msg->data[1];
            RCLCPP_DEBUG(this->get_logger(),
                "Motor command: left=%.2f, right=%.2f", duty_left, duty_right);
            // TODO: set_pwm(GPIO_MOTOR_LEFT, duty_left);
            // TODO: set_pwm(GPIO_MOTOR_RIGHT, duty_right);
            // TODO: kick_watchdog();
        }
    }
    
    void hardware_read_loop() {
        // TODO: Main hardware I/O loop (100 Hz minimum)
        //   1. Read encoder counts via interrupt-safe snapshot
        //   2. Read IMU via I2C (SPI fallback)
        //   3. Read ADC for battery voltage
        //   4. Check watchdog, handle safety shutdown
        //   5. Publish all sensor data on ROS topics
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LowLevelInterface>());
    rclcpp::shutdown();
    return 0;
}
