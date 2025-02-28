#ifndef BOT_SDK_CAN_SENDER_HPP_
#define BOT_SDK_CAN_SENDER_HPP_

#include <chrono>
#include <cstdint>
#include <string>

#include "bot_sdk_can/can_id.hpp"
#include "bot_sdk_can/visibility_control.hpp"

namespace bot {
namespace can {

class BOT_PUBLIC CanSender {
 public:
  explicit CanSender(const std::string &interface = "can0",
                     const bool enable_fd = false,
                     const CanId &default_id = CanId{});
  ~CanSender() noexcept;

  /// Send raw data with the default id
  void Send(const void *const data, const std::size_t length,
            const std::chrono::nanoseconds timeout =
                std::chrono::nanoseconds::zero()) const;
  /// Send raw data with an explicit CAN id
  void Send(const void *const data, const std::size_t length, const CanId id,
            const std::chrono::nanoseconds timeout =
                std::chrono::nanoseconds::zero()) const;
  /// Send typed data with the default id
  template <typename T, typename = std::enable_if_t<!std::is_pointer<T>::value>>
  void Send(const T &data, const std::chrono::nanoseconds timeout =
                               std::chrono::nanoseconds::zero()) const {
    Send(data, default_id_, timeout);
  }

  /// Send typed data with an explicit CAN Id
  template <typename T, typename = std::enable_if_t<!std::is_pointer<T>::value>>
  void Send(const T &data, const CanId id,
            const std::chrono::nanoseconds timeout =
                std::chrono::nanoseconds::zero()) const {
    static_assert(sizeof(data) <= MAX_DATA_LENGTH,
                  "Data type too large for CAN");

    SendImpl(reinterpret_cast<const char *>(&data), sizeof(data), id, timeout);
  }

  /// Send raw data with the default id
  void SendFd(const void *const data, const std::size_t length,
              const std::chrono::nanoseconds timeout =
                  std::chrono::nanoseconds::zero()) const;

  /// Send raw data with an explicit CAN id
  void SendFd(const void *const data, const std::size_t length, const CanId id,
              const std::chrono::nanoseconds timeout =
                  std::chrono::nanoseconds::zero()) const;

  /// Send typed data with the default id
  template <typename T, typename = std::enable_if_t<!std::is_pointer<T>::value>>
  void SendFd(const T &data, const std::chrono::nanoseconds timeout =
                                 std::chrono::nanoseconds::zero()) const {
    SendFd(data, default_id_, timeout);
  }

  /// Send typed data with an explicit CAN Id
  template <typename T, typename = std::enable_if_t<!std::is_pointer<T>::value>>
  void SendFd(const T &data, const CanId id,
              const std::chrono::nanoseconds timeout =
                  std::chrono::nanoseconds::zero()) const {
    static_assert(sizeof(data) <= MAX_FD_DATA_LENGTH,
                  "Data type too large for CAN FD");
    SendFdImpl(reinterpret_cast<const char *>(&data), sizeof(data), id,
               timeout);
  }

  /// Get the default CAN id
  CanId default_id() const noexcept;

 private:
  // Underlying implementation of sending, data is assumed to be of an
  // appropriate length
  void SendImpl(const void *const data, const std::size_t length,
                const CanId id, const std::chrono::nanoseconds timeout) const;

  // Underlying implementation of FD sending, data is assumed to be of an
  // appropriate length
  void SendFdImpl(const void *const data, const std::size_t length,
                  const CanId id, const std::chrono::nanoseconds timeout) const;

  // Wait for file descriptor to be available to send data via select()
  BOT_LOCAL void Wait(const std::chrono::nanoseconds timeout) const;

  bool enable_fd_;
  int32_t file_descriptor_{};
  CanId default_id_;
};  // class CanSender

}  // namespace can
}  // namespace bot

#endif  // BOT_SDK_CAN_SENDER_HPP_
