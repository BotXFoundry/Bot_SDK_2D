#include "bot_sdk_can/can_sender_node.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "bot_sdk_can/can_common.hpp"

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using lifecycle_msgs::msg::State;

namespace bot {
namespace can {
CanSenderNode::CanSenderNode(rclcpp::NodeOptions options)
    : lc::LifecycleNode("can_sender_node", options) {
  interface_ = this->declare_parameter("interface", "can0");
  enable_fd_ = this->declare_parameter("enable_can_fd", false);
  double timeout_sec = this->declare_parameter("timeout_sec", 0.01);
  timeout_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(timeout_sec));

  RCLCPP_INFO(this->get_logger(), "interface: %s", interface_.c_str());
  RCLCPP_INFO(this->get_logger(), "can fd enabled: %s",
              enable_fd_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "timeout(s): %f", timeout_sec);
}

LNI::CallbackReturn CanSenderNode::on_configure(const lc::State& state) {
  (void)state;

  try {
    sender_ = std::make_unique<CanSender>(interface_, enable_fd_);
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(this->get_logger(), "Error opening CAN sender: %s - %s",
                 interface_.c_str(), ex.what());
    return LNI::CallbackReturn::FAILURE;
  }

  RCLCPP_DEBUG(this->get_logger(), "Sender successfully configured.");

  if (!enable_fd_) {
    can_subscriber_ = this->create_subscription<bot_sdk_msgs::msg::Frame>(
        "can", 500,
        std::bind(&CanSenderNode::OnFrame, this, std::placeholders::_1));
  } else {
    fdcan_subscriber_ = this->create_subscription<bot_sdk_msgs::msg::FdFrame>(
        "fdcan", 500,
        std::bind(&CanSenderNode::OnFdFrame, this, std::placeholders::_1));
  }
  // 订阅 cmd_vel 话题
  velocity_command_subscriber_ =
      this->create_subscription<geometry_msgs::msg::Twist>(
          "velocity_command", 10,
          std::bind(&CanSenderNode::OnVelocityCommand, this,
                    std::placeholders::_1));

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn CanSenderNode::on_activate(const lc::State& state) {
  (void)state;
  RCLCPP_DEBUG(this->get_logger(), "Sender activated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn CanSenderNode::on_deactivate(const lc::State& state) {
  (void)state;
  RCLCPP_DEBUG(this->get_logger(), "Sender deactivated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn CanSenderNode::on_cleanup(const lc::State& state) {
  (void)state;

  if (!enable_fd_) {
    can_subscriber_.reset();
  } else {
    fdcan_subscriber_.reset();
  }
  velocity_command_subscriber_.reset();
  RCLCPP_DEBUG(this->get_logger(), "Sender cleaned up.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn CanSenderNode::on_shutdown(const lc::State& state) {
  (void)state;
  RCLCPP_DEBUG(this->get_logger(), "Sender shutting down.");
  return LNI::CallbackReturn::SUCCESS;
}

void CanSenderNode::OnFrame(const bot_sdk_msgs::msg::Frame::SharedPtr msg) {
  if (this->get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
    FrameType type;
    if (msg->is_rtr) {
      type = FrameType::REMOTE;
    } else if (msg->is_error) {
      type = FrameType::ERROR;
    } else {
      type = FrameType::DATA;
    }

    CanId send_id = msg->is_extended ? CanId(msg->id, 0, type, ExtendedFrame)
                                     : CanId(msg->id, 0, type, StandardFrame);
    try {
      sender_->Send(msg->data.data(), msg->dlc, send_id, timeout_ns_);
    } catch (const std::exception& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Error sending CAN message: %s - %s",
                           interface_.c_str(), ex.what());
      return;
    }
  }
}

void CanSenderNode::OnFdFrame(const bot_sdk_msgs::msg::FdFrame::SharedPtr msg) {
  if (this->get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
    FrameType type;
    if (msg->is_error) {
      type = FrameType::ERROR;
    } else {
      type = FrameType::DATA;
    }

    CanId send_id = msg->is_extended ? CanId(msg->id, 0, type, ExtendedFrame)
                                     : CanId(msg->id, 0, type, StandardFrame);
    try {
      sender_->SendFd(msg->data.data<void>(), msg->len, send_id, timeout_ns_);
    } catch (const std::exception& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Error sending CAN message: %s - %s",
                           interface_.c_str(), ex.what());
      return;
    }
  }
}

void CanSenderNode::OnVelocityCommand(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (this->get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
    // 将速度指令转换为CAN数据帧
    CanId send_id(0x114, 0, FrameType::DATA, StandardFrame);  // 示例ID
    std::vector<int8_t> data(7, 0);

    int16_t linear = static_cast<int16_t>(msg->linear.x * 1000);
    int16_t angular = static_cast<int16_t>(msg->angular.z * 100);

    data[0] = 0x01;
    data[1] = (linear >> 8) & 0xFF;  // 车体线速度高八位
    data[2] = linear & 0xFF;         // 车体线速度低八位
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = (angular >> 8) & 0xFF;  // 车体角速度高八位
    data[6] = angular & 0xFF;         // 车体角速度低八位

    try {
      sender_->Send(data.data(), data.size(), send_id, timeout_ns_);
    } catch (const std::exception& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Error sending CAN message: %s - %s",
                           interface_.c_str(), ex.what());
      return;
    }
  }
}

}  // namespace can
}  // namespace bot

RCLCPP_COMPONENTS_REGISTER_NODE(bot::can::CanSenderNode)
