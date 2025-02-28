#ifndef BOT_SDK_CAN_COMMON_HPP_
#define BOT_SDK_CAN_COMMON_HPP_

#include <linux/can.h>
#include <sys/select.h>
#include <sys/time.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace bot {
namespace can {

/// Bind a non-blocking CAN_RAW socket to the given interface
int32_t BindCanSocket(const std::string& interface, bool enable_fd);

/// Set SocketCAN filters
void SetCanFilter(int32_t fd, const std::vector<struct can_filter>& f_list);

/// Set SocketCAN error filter
void SetCanErrorFilter(int32_t fd, can_err_mask_t err_mask);

/// Set filters joining option for SocketCAN. If set, all filters
/// must match for the frame to be passed.
void SetCanFilterJoin(int32_t fd, bool join_filters);

/// Convert std::chrono duration to timeval (with microsecond resolution)
struct timeval ToTimeval(const std::chrono::nanoseconds timeout) noexcept;
/// Convert timeval to time in microseconds
uint64_t FromTimeval(const struct timeval tv) noexcept;
/// Create a fd_set for use with select() that only contains the specified file
/// descriptor
fd_set SingleSet(int32_t file_descriptor) noexcept;

}  // namespace can
}  // namespace bot

#endif  // BOT_SDK_CAN_COMMON_HPP_
