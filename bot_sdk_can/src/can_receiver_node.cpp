#include "bot_sdk_can/can_receiver_node.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "bot_sdk_can/can_common.hpp"

namespace LC = rclcpp_lifecycle;  // 生命周期相关的命名空间
using LNI = rclcpp_lifecycle::node_interfaces::
    LifecycleNodeInterface;        // 生命周期节点接口
using lifecycle_msgs::msg::State;  // 生命周期消息的状态
using namespace std::chrono_literals;  // 使用秒、毫秒等时间单位的字面量

namespace bot {
namespace can {
CanReceiverNode::CanReceiverNode(rclcpp::NodeOptions options)
    : LC::LifecycleNode("can_receiver_node", options) {
  // 从参数服务器获取参数
  interface_ = this->declare_parameter("interface", "can0");
  use_bus_time_ = this->declare_parameter<bool>("use_bus_time", false);
  enable_fd_ = this->declare_parameter<bool>("enable_can_fd", true);
  double interval_sec =
      this->declare_parameter("interval_sec", 0.01);  // 使用定时器的间隔时间
  this->declare_parameter("filters", "0:0");
  interval_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(interval_sec));

  // 初始化定时器
  /*publish_timer_= this->create_wall_timer(std::chrono::milliseconds(1),//10ms
                                          [this](){this->receive();});*/

  // 打印配置信息
  RCLCPP_INFO(this->get_logger(), "interface: %s", interface_.c_str());
  RCLCPP_INFO(this->get_logger(), "use bus time: %d", use_bus_time_);
  RCLCPP_INFO(this->get_logger(), "can fd enabled: %s",
              enable_fd_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "interval(s): %f", interval_sec);
}

LNI::CallbackReturn CanReceiverNode::on_configure(const LC::State& state) {
  (void)state;

  try {
    // 初始化 CAN 接收器
    receiver_ = std::make_unique<CanReceiver>(interface_, enable_fd_);
    // 应用 CAN 过滤器
    auto filters = get_parameter("filters").as_string();
    receiver_->SetCanFilters(CanReceiver::CanFilterList(filters));
    RCLCPP_INFO(get_logger(), "applied filters: %s", filters.c_str());
  } catch (const std::exception& ex) {
    // 捕获并处理异常
    RCLCPP_ERROR(this->get_logger(), "Error opening CAN receiver: %s - %s",
                 interface_.c_str(), ex.what());
    return LNI::CallbackReturn::FAILURE;
  }

  RCLCPP_DEBUG(this->get_logger(), "Receiver successfully configured.");

  // 根据是否启用 CAN FD 创建相应的发布者
  if (!enable_fd_) {
    can_frame_publisher_ =
        this->create_publisher<bot_sdk_msgs::msg::Frame>("from_can_bus", 500);
  } else {
    fdcan_frame_publisher_ = this->create_publisher<bot_sdk_msgs::msg::FdFrame>(
        "from_can_bus_fd", 500);
  }

  // 启动接收线程
  receiver_thread_ =
      std::make_unique<std::thread>(&CanReceiverNode::receive, this);

  // 初始化IMU话题
  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

  // 初始化左右轮编码器脉冲数话题
  odometry_publisher1_ =
      this->create_publisher<std_msgs::msg::Int32>("odometry1", 10);
  odometry_publisher2_ =
      this->create_publisher<std_msgs::msg::Int32>("odometry2", 10);

  // 初始化脉冲数差值
  encoder_publisher1_ =
      this->create_publisher<std_msgs::msg::Int32>("encoder1", 10);
  encoder_publisher2_ =
      this->create_publisher<std_msgs::msg::Int32>("encoder2", 10);
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn CanReceiverNode::on_activate(const LC::State& state) {
  (void)state;

  // 激活相应的发布者
  if (!enable_fd_) {
    can_frame_publisher_->on_activate();
  } else {
    fdcan_frame_publisher_->on_activate();
  }
  imu_publisher_->on_activate();        // 激活 IMU 发布者
  odometry_publisher1_->on_activate();  // 激活左轮编码器数据发布者
  odometry_publisher2_->on_activate();  // 激活右轮编码器数据发布者

  encoder_publisher1_->on_activate();  // 激活左轮编码器差值数据发布者
  encoder_publisher2_->on_activate();  // 激活右轮编码器差值数据发布者

  RCLCPP_DEBUG(this->get_logger(), "Receiver activated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn CanReceiverNode::on_deactivate(const LC::State& state) {
  (void)state;

  // 停用相应的发布者
  if (!enable_fd_) {
    can_frame_publisher_->on_deactivate();
  } else {
    fdcan_frame_publisher_->on_deactivate();
  }

  imu_publisher_->on_deactivate();
  odometry_publisher1_->on_deactivate();
  odometry_publisher2_->on_deactivate();
  encoder_publisher1_->on_deactivate();
  encoder_publisher2_->on_deactivate();

  RCLCPP_DEBUG(this->get_logger(), "Receiver deactivated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn CanReceiverNode::on_cleanup(const LC::State& state) {
  (void)state;

  // 释放发布者和停止接收线程
  if (!enable_fd_) {
    can_frame_publisher_.reset();
  } else {
    fdcan_frame_publisher_.reset();
  }
  imu_publisher_.reset();
  odometry_publisher1_.reset();
  odometry_publisher2_.reset();
  encoder_publisher1_.reset();
  encoder_publisher2_.reset();
  if (receiver_thread_->joinable()) {
    receiver_thread_->join();
  }
  RCLCPP_DEBUG(this->get_logger(), "Receiver cleaned up.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn CanReceiverNode::on_shutdown(const LC::State& state) {
  (void)state;
  RCLCPP_DEBUG(this->get_logger(), "Receiver shutting down.");
  return LNI::CallbackReturn::SUCCESS;
}

void CanReceiverNode::Receive() {
  CanId receive_id{};
  std::vector<uint8_t> data_buffer(8);  // 创建一个缓冲区来存储CAN数据

  if (!enable_fd_) {
    bot_sdk_msgs::msg::Frame frame_msg(
        rosidl_runtime_cpp::MessageInitialization::ZERO);
    frame_msg.header.frame_id = "can";
    auto last_imu_publish_time = this->now();
    while (rclcpp::ok()) {
      if (this->get_current_state().id() != State::PRIMARY_STATE_ACTIVE) {
        std::this_thread::sleep_for(100ms);
        continue;
      }

      try {
        receive_id = receiver_->Receive(data_buffer.data(), interval_ns_);
        std::cout << "data: ";
        for (auto byte : data_buffer) {
          std::cout << std::hex << static_cast<int>(byte) << " ";
        }
        std::cout << std::endl;
      } catch (const std::exception& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Error receiving CAN message: %s - %s",
                             interface_.c_str(), ex.what());
        continue;
      }

      if (use_bus_time_) {
        frame_msg.header.stamp =
            rclcpp::Time(static_cast<int64_t>(receive_id.GetBusTime() * 1000U));
      } else {
        frame_msg.header.stamp = this->now();
      }

      frame_msg.id = receive_id.GetId();
      frame_msg.is_rtr = (receive_id.GetFrameType() == FrameType::REMOTE);
      frame_msg.is_extended = receive_id.IsExtended();
      frame_msg.is_error = (receive_id.GetFrameType() == FrameType::ERROR);
      frame_msg.dlc = receive_id.Length();
      can_frame_publisher_->publish(std::move(frame_msg));

      // Check CAN ID and publish IMU data if applicable
      switch (receive_id.GetId()) {
        case 0x261:
          PublishImuAcceleration(receive_id, data_buffer);
          break;
        case 0x262:
          PublishImuAngular(receive_id, data_buffer);
          break;
        case 0x191:
          PublishEncoderMotor1(receive_id, data_buffer);
          break;
        case 0x193:
          PublishEncoderMotor2(receive_id, data_buffer);
          break;

        default:
      }
    }
  } else {
    bot_sdk_msgs::msg::FdFrame fd_frame_msg(
        rosidl_runtime_cpp::MessageInitialization::ZERO);
    fd_frame_msg.header.frame_id = "can";

    while (rclcpp::ok()) {
      if (this->get_current_state().id() != State::PRIMARY_STATE_ACTIVE) {
        std::this_thread::sleep_for(100ms);
        continue;
      }

      fd_frame_msg.data.resize(64);

      try {
        receive_id =
            receiver_->ReceiveFd(fd_frame_msg.data.data<void>(), interval_ns_);
      } catch (const std::exception& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Error receiving CAN FD message: %s - %s",
                             interface_.c_str(), ex.what());
        continue;
      }

      fd_frame_msg.data.resize(receive_id.Length());

      if (use_bus_time_) {
        fd_frame_msg.header.stamp =
            rclcpp::Time(static_cast<int64_t>(receive_id.GetBusTime() * 1000U));
      } else {
        fd_frame_msg.header.stamp = this->now();
      }

      fd_frame_msg.id = receive_id.GetId();
      fd_frame_msg.is_extended = receive_id.IsExtended();
      fd_frame_msg.is_error = (receive_id.GetFrameType() == FrameType::ERROR);
      fd_frame_msg.len = receive_id.Length();
      fdcan_frame_publisher_->publish(std::move(fd_frame_msg));
    }
  }
}
// 加速度计
void CanReceiverNode::PublishImuAcceleration(const CanId& receive_id,
                                             const std::vector<uint8_t>& data) {
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.frame_id = "imu_link";
  imu_msg.header.stamp = this->now();

  // 解析加速度计数据
  if (data.size() >= 8) {  // 确保数据长度足够
    // 解析 X 轴加速度
    int32_t raw_acc_x = (data[0] << 16) | (data[1] << 8) | data[2];
    // 处理负数
    if (raw_acc_x & 0x00800000) {  // 检查最高位
      raw_acc_x |= 0xFF000000;     // 扩展符号位
    }
    imu_msg.linear_acceleration.x = static_cast<float>(raw_acc_x) / 30000.0f;

    // 解析 Y 轴加速度
    int32_t raw_acc_y = (data[3] << 16) | (data[4] << 8) | data[5];
    if (raw_acc_y & 0x00800000) {
      raw_acc_y |= 0xFF000000;
    }
    imu_msg.linear_acceleration.y = static_cast<float>(raw_acc_y) / 30000.0f;

    // 解析 Z 轴加速度
    int16_t raw_acc_z = (data[6] << 8) | data[7];
    if (raw_acc_z & 0x8000) {   // 检查最高位
      raw_acc_z |= 0xFFFF0000;  // 扩展符号位
    }
    imu_msg.linear_acceleration.z = static_cast<float>(raw_acc_z) / 1500.0f;
  }

  // 发布IMU消息
  imu_publisher_->publish(imu_msg);
}

// 解析陀螺仪数据
void CanReceiverNode::PublishImuAngular(const CanId& receive_id,
                                        const std::vector<uint8_t>& data) {
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.frame_id = "imu_link";
  imu_msg.header.stamp = this->now();

  if (data.size() >= 8) {  // 确保数据长度足够
    // 解析 X 轴角速度
    int32_t raw_vel_x = (data[0] << 16) | (data[1] << 8) | data[2];
    // 处理负数
    if (raw_vel_x & 0x00800000) {  // 检查最高位
      raw_vel_x |= 0xFF000000;     // 扩展符号位
    }
    imu_msg.angular_velocity.x = static_cast<float>(raw_vel_x) / 30000.0f;

    // 解析 Y 轴角速度
    int32_t raw_vel_y = (data[3] << 16) | (data[4] << 8) | data[5];
    if (raw_vel_y & 0x00800000) {
      raw_vel_y |= 0xFF000000;
    }
    imu_msg.angular_velocity.y = static_cast<float>(raw_vel_y) / 30000.0f;

    // 解析 Z 轴角速度
    int16_t raw_vel_z = (data[6] << 8) | data[7];
    if (raw_vel_z & 0x8000) {   // 检查最高位
      raw_vel_z |= 0xFFFF0000;  // 扩展符号位
    }
    imu_msg.angular_velocity.z = static_cast<float>(raw_vel_z) / 1500.0f;
  }

  // 发布IMU消息
  imu_publisher_->publish(imu_msg);
}

// 解析电机1编码器数据
void CanReceiverNode::PublishEncoderMotor1(const CanId& receive_id,
                                           const std::vector<uint8_t>& data) {
  if (data.size() >= 8) {  // 确保数据长度足够
    int32_t pulse_count = static_cast<int32_t>(data[0]) |
                          (static_cast<int32_t>(data[1]) << 8) |
                          (static_cast<int32_t>(data[2]) << 16) |
                          (static_cast<int32_t>(data[3]) << 24);
    int32_t d_count =
        static_cast<int32_t>(data[4]) | (static_cast<int32_t>(data[5] << 8));
    // 创建并发布左轮编码器消息
    std_msgs::msg::Int32 odometry_msg;
    odometry_msg.data = pulse_count;
    odometry_publisher1_->publish(odometry_msg);

    std_msgs::msg::Int32 encoder_msg;
    encoder_msg.data = d_count;
    encoder_publisher1_->publish(encoder_msg);
  }
}

// 解析电机2编码器数据
void CanReceiverNode::PublishEncoderMotor2(const CanId& receive_id,
                                           const std::vector<uint8_t>& data) {
  // 解析电机2编码器数据
  if (data.size() >= 8) {  // 确保数据长度足够
    int32_t pulse_count = static_cast<int32_t>(data[0]) |
                          (static_cast<int32_t>(data[1]) << 8) |
                          (static_cast<int32_t>(data[2]) << 16) |
                          (static_cast<int32_t>(data[3]) << 24);
    int32_t d_count =
        static_cast<int32_t>(data[4]) | (static_cast<int32_t>(data[5] << 8));
    // 创建并发布右轮编码器消息
    std_msgs::msg::Int32 odometry_msg;
    odometry_msg.data = pulse_count;
    odometry_publisher2_->publish(odometry_msg);

    std_msgs::msg::Int32 encoder_msg;
    encoder_msg.data = d_count;
    encoder_publisher2_->publish(encoder_msg);
  }
}

}  // namespace can
}  // namespace bot

RCLCPP_COMPONENTS_REGISTER_NODE(bot::can::CanReceiverNode)
