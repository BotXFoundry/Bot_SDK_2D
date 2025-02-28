#ifndef BOT_SDK_CAN_RECEIVER_HPP_
#define BOT_SDK_CAN_RECEIVER_HPP_

#include <linux/can.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "bot_sdk_can/can_id.hpp"
#include "bot_sdk_can/visibility_control.hpp"

namespace bot {
namespace can {

class BOT_PUBLIC CanReceiver {
 public:
  explicit CanReceiver(const std::string& interface = "can0",
                       const bool enable_fd = false);
  ~CanReceiver() noexcept;

  struct CanFilterList {
    std::vector<struct can_filter> filters;
    can_err_mask_t error_mask = 0;
    bool join_filters = false;

    CanFilterList() = default;

    explicit CanFilterList(const char* str);

    explicit CanFilterList(const std::string& str);

    static CanFilterList ParseFilters(const std::string& str);
  };

  /// Set SocketCAN filters
  void SetCanFilters(const CanFilterList& filters);

  /// Receive CAN data
  CanId Receive(void* const data, const std::chrono::nanoseconds timeout =
                                      std::chrono::nanoseconds::zero()) const;
  /// Receive typed CAN data.
  template <typename T, typename = std::enable_if_t<!std::is_pointer<T>::value>>
  CanId Receive(T& data, const std::chrono::nanoseconds timeout =
                             std::chrono::nanoseconds::zero()) const {
    static_assert(sizeof(data) <= MAX_DATA_LENGTH,
                  "Data type too large for CAN");
    std::array<uint8_t, MAX_DATA_LENGTH> data_raw{};
    const auto ret = Receive(&data_raw[0U], timeout);
    if (ret.Length() != sizeof(data)) {
      throw std::runtime_error{
          "Received CAN data is of size incompatible with provided type!"};
    }
    (void)std::memcpy(&data, &data_raw[0U], ret.Length());
    return ret;
  }

  /// Receive CAN FD data
  CanId ReceiveFd(void* const data,
                   const std::chrono::nanoseconds timeout =
                       std::chrono::nanoseconds::zero()) const;
  /// Receive typed CAN FD data.
  template <typename T, typename = std::enable_if_t<!std::is_pointer<T>::value>>
  CanId ReceiveFd(T& data, const std::chrono::nanoseconds timeout =
                                std::chrono::nanoseconds::zero()) const {
    static_assert(sizeof(data) <= MAX_FD_DATA_LENGTH,
                  "Data type too large for CAN FD");
    std::array<uint8_t, MAX_FD_DATA_LENGTH> data_raw{};
    const auto ret = ReceiveFd(&data_raw[0U], timeout);
    if (ret.Length() != sizeof(data)) {
      throw std::runtime_error{
          "Received CAN FD data is of size incompatible with provided type!"};
    }
    (void)std::memcpy(&data, &data_raw[0U], ret.Length());
    return ret;
  }

 private:
  // Wait for file descriptor to be available to send data via select()
  BOT_LOCAL void Wait(const std::chrono::nanoseconds timeout) const;

  int32_t file_descriptor_;
  bool enable_fd_;
};  // class CanReceiver

}  // namespace can
}  // namespace bot

#endif  // BOT_SDK_CAN_RECEIVER_HPP_
