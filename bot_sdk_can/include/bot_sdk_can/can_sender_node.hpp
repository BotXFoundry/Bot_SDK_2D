#ifndef BOT_SDK_CAN_SENDER_NODE_HPP_
#define BOT_SDK_CAN_SENDER_NODE_HPP_

#include <memory>
#include <string>

#include "bot_sdk_can/can_sender.hpp"
#include "bot_sdk_can/visibility_control.hpp"
#include "bot_sdk_msgs/msg/fd_frame.hpp"
#include "bot_sdk_msgs/msg/frame.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"

namespace LC = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace bot {
namespace can {

class BOT_PUBLIC CanSenderNode final : public LC::LifecycleNode {
 public:
  explicit CanSenderNode(rclcpp::NodeOptions options);

  // Callback from transition to "configuring" state.
  LNI::CallbackReturn on_configure(const LC::State& state) override;

  // Callback from transition to "activating" state.
  LNI::CallbackReturn on_activate(const LC::State& state) override;

  // Callback from transition to "deactivating" state.
  LNI::CallbackReturn on_deactivate(const LC::State& state) override;

  LNI::CallbackReturn on_cleanup(const LC::State& state) override;

  // Callback from transition to "shutdown" state.
  LNI::CallbackReturn on_shutdown(const LC::State& state) override;

  // Callback for ros can frame.
  void OnFrame(const bot_sdk_msgs::msg::Frame::SharedPtr msg);

  // Callback for ros can fd frame.
  void OnFdFrame(const bot_sdk_msgs::msg::FdFrame::SharedPtr msg);

  // Callback for cmd_vel messages.
  void OnVelocityCommand(
      const geometry_msgs::msg::Twist::SharedPtr msg);  // 添加回调函数声明

 private:
  std::string interface_;
  bool enable_fd_;
  rclcpp::Subscription<bot_sdk_msgs::msg::Frame>::SharedPtr can_subscriber_;
  rclcpp::Subscription<bot_sdk_msgs::msg::FdFrame>::SharedPtr fdcan_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      velocity_command_subscriber_;  // 添加订阅成员变量
  std::unique_ptr<CanSender> sender_;
  std::chrono::nanoseconds timeout_ns_;
};
}  // namespace can
}  // namespace bot

#endif  // BOT_SDK_CAN_SENDER_NODE_HPP_
