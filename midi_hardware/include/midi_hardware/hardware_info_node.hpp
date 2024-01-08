/**
 * @file hardware_info_node.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef HARDWARE_INFO_NODE_HPP_
#define HARDWARE_INFO_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>

#include "midi_custom_interfaces/msg/encoder_data.hpp"
#include "midi_custom_interfaces/msg/motor_command.hpp"
#include "midi_hardware/midi_hardware_defs.hpp"

class HardwareInfoNode : public rclcpp::Node
{
public:
    HardwareInfoNode();

    ~HardwareInfoNode() noexcept
    {

    }

    bool init();

    const midi_custom_interfaces::msg::EncoderData getEncoderData();

    bool sendCommands(const midi_custom_interfaces::msg::MotorCommand& motor_command_msg);

    inline const HardwareParams getHardwareParams() const
    {  
        return m_HwParams;
    }

private:

    rclcpp::Publisher<midi_custom_interfaces::msg::MotorCommand>::SharedPtr m_MotorCommandPub;
    std::shared_ptr<realtime_tools::RealtimePublisher<midi_custom_interfaces::msg::MotorCommand>> m_RtMotorCommandPub;

    rclcpp::Subscription<midi_custom_interfaces::msg::EncoderData>::SharedPtr m_EncoderDataSub;
    realtime_tools::RealtimeBuffer<midi_custom_interfaces::msg::EncoderData> m_EncoderDataBuffer;

    HardwareParams m_HwParams;

};

#endif // HARDWARE_INFO_NODE_HPP_