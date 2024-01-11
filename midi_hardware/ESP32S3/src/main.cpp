#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <midi_custom_interfaces/msg/encoder_data.h>
#include <midi_custom_interfaces/msg/motor_command.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#define IN1 4
#define IN2 5
#define VREF 6
#define NSLEEP 7

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

midi_custom_interfaces__msg__EncoderData encoder_data_msg;
midi_custom_interfaces__msg__MotorCommand motor_cmd_msg;
rclc_executor_t executor;

void motorCmdCallback(const void *msgin)
{
    const midi_custom_interfaces__msg__MotorCommand *cmd_msg = (const midi_custom_interfaces__msg__MotorCommand *)msgin;
    motor_cmd_msg = *cmd_msg;
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(VREF, OUTPUT);
    pinMode(NSLEEP, OUTPUT);
    set_microros_serial_transports(Serial);

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rclc_support_t support;
    rcl_allocator_t allocator = rcl_get_default_allocator();

    rclc_support_init(&support, 0, NULL, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, "esp32_hardware_node", "", &support);

    rcl_publisher_t encoder_info_pub;
    rclc_publisher_init_default(
        &encoder_info_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(midi_custom_interfaces, msg, EncoderData),
        "encoder_data_pub");

    rcl_subscription_t motor_command_sub;
    rclc_subscription_init_default(
        &motor_command_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(midi_custom_interfaces, msg, MotorCommand),
        "motor_command_sub");

    rclc_executor_init(&executor, &support.context, 1, &allocator);

    rclc_executor_add_subscription(
        &executor,
        &motor_command_sub,
        &motor_cmd_msg,
        &motorCmdCallback,
        ON_NEW_DATA);
}

void loop()
{
    delay(100);
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
