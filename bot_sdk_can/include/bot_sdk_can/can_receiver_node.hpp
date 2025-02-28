#ifndef BOT_SDK_CAN_RECEIVER_NODE_HPP_
#define BOT_SDK_CAN_RECEIVER_NODE_HPP_

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "bot_sdk_can/can_receiver.hpp"
#include "bot_sdk_can/visibility_control.hpp"
#include "bot_sdk_msgs/msg/fd_frame.hpp"
#include "bot_sdk_msgs/msg/frame.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int32.hpp"
namespace LC = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace bot {
namespace can {

class BOT_PUBLIC CanReceiverNode final : public LC::LifecycleNode {
 public:
  explicit CanReceiverNode(rclcpp::NodeOptions options);

  // Callback from transition to "configuring" state.
  LNI::CallbackReturn on_configure(const LC::State& state) override;

  // Callback from transition to "activating" state.
  LNI::CallbackReturn on_activate(const LC::State& state) override;

  // Callback from transition to "deactivating" state.
  LNI::CallbackReturn on_deactivate(const LC::State& state) override;

  // Callback from transition to "unconfigured" state.
  LNI::CallbackReturn on_cleanup(const LC::State& state) override;

  // Callback from transition to "shutdown" state.
  LNI::CallbackReturn on_shutdown(const LC::State& state) override;

  // Callback for reading from hardware interface on timer tick.
  void Receive();
  // 加速度计
  void PublishImuAcceleration(const CanId& receive_id,
                              const std::vector<uint8_t>& data);
  // 陀螺仪
  void PublishImuAngular(const CanId& receive_id,
                         const std::vector<uint8_t>& data);
  // 电机1编码器数据
  void PublishEncoderMotor1(const CanId& receive_id,
                            const std::vector<uint8_t>& data);
  // 电机2编码器数据
  void PublishEncoderMotor2(const CanId& receive_id,
                            const std::vector<uint8_t>& data);

 private:
  std::string interface_;
  std::shared_ptr<LC::LifecyclePublisher<bot_sdk_msgs::msg::Frame>>
      can_frame_publisher_;
  std::shared_ptr<LC::LifecyclePublisher<bot_sdk_msgs::msg::FdFrame>>
      fdcan_frame_publisher_;
  std::shared_ptr<LC::LifecyclePublisher<sensor_msgs::msg::Imu>> imu_publisher_;
  std::shared_ptr<LC::LifecyclePublisher<std_msgs::msg::Int32>>
      odometry_publisher1_;
  std::shared_ptr<LC::LifecyclePublisher<std_msgs::msg::Int32>>
      odometry_publisher2_;

  std::shared_ptr<LC::LifecyclePublisher<std_msgs::msg::Int32>>
      encoder_publisher1_;
  std::shared_ptr<LC::LifecyclePublisher<std_msgs::msg::Int32>>
      encoder_publisher2_;

  std::unique_ptr<CanReceiver> receiver_;
  std::unique_ptr<std::thread> receiver_thread_;
  std::chrono::nanoseconds interval_ns_;
  bool enable_fd_;
  bool use_bus_time_;
  std::shared_ptr<rclcpp::TimerBase> publish_timer_;
};
}  // namespace can
}  // namespace bot

#endif  // BOT_SDK_CAN_RECEIVER_NODE_HPP_
