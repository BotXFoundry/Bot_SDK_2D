#ifndef BOT_SDK_CAN_ID_HPP_
#define BOT_SDK_CAN_ID_HPP_

#include <cstdint>
#include <stdexcept>

#include "bot_sdk_can/visibility_control.hpp"

namespace bot {
namespace can {

constexpr std::size_t MAX_DATA_LENGTH = 8U;
constexpr std::size_t MAX_FD_DATA_LENGTH = 64U;

class BOT_PUBLIC SocketCanTimeout : public std::runtime_error {
 public:
  explicit SocketCanTimeout(const char* const what) : runtime_error{what} {}
};

enum class FrameType : uint32_t { DATA, ERROR, REMOTE };

struct StandardFrame_ {};

constexpr StandardFrame_ StandardFrame;

struct ExtendedFrame_ {};

constexpr ExtendedFrame_ ExtendedFrame;

class BOT_PUBLIC CanId {
 public:
  CanId() = default;

  explicit CanId(const uint32_t raw_id, const uint64_t bus_time,
                 const uint32_t data_length = 0U);

  CanId(const uint32_t id, const uint64_t bus_time, FrameType type,
        StandardFrame_);

  CanId(const uint32_t id, const uint64_t bus_time, FrameType type,
        ExtendedFrame_);

  /// Sets bit 31 to 0
  CanId& Standard() noexcept;
  /// Sets bit 31 to 1
  CanId& Extended() noexcept;
  /// Sets bit 29 to 1, and bit 30 to 0
  CanId& ErrorFrame() noexcept;
  /// Sets bit 29 to 0, and bit 30 to 1
  CanId& RemoteFrame() noexcept;
  /// Clears bits 29 and 30 (sets to 0)
  CanId& DataFrame() noexcept;
  /// Sets the type accordingly
  CanId& SetFrameType(const FrameType type);
  /// Sets leading bits
  CanId& SetId(const uint32_t id);

  /// Get just the can_id bits
  uint32_t GetId() const noexcept;
  /// Get the whole id value
  uint32_t Get() const noexcept;
  /// Check if frame is extended
  bool IsExtended() const noexcept;
  /// Check frame type
  FrameType GetFrameType() const;
  /// Get the length of the data; only nonzero on received data
  uint32_t Length() const noexcept;

  uint64_t GetBusTime() { return bus_time_; }

 private:
  BOT_LOCAL CanId(const uint32_t id, const uint64_t bus_time, FrameType type,
                  bool is_extended);

  uint32_t id_{};
  uint32_t data_length_{};
  uint64_t bus_time_;
};  // class CanId
}  // namespace can
}  // namespace bot

#endif  // BOT_SDK_CAN_ID_HPP_
