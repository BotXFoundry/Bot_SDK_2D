#include "bot_sdk_can/can_sender.hpp"

#include <linux/can.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>

#include "bot_sdk_can/can_common.hpp"

namespace bot {
namespace can {

CanSender::CanSender(const std::string &interface, const bool enable_fd,
                     const CanId &default_id)
    : enable_fd_(enable_fd),
      file_descriptor_{BindCanSocket(interface, enable_fd_)},
      default_id_{default_id} {}

CanSender::~CanSender() noexcept {
  (void)close(file_descriptor_);
}

CanId CanSender::default_id() const noexcept { return default_id_; }

void CanSender::Send(const void *const data, const std::size_t length,
                     const CanId id,
                     const std::chrono::nanoseconds timeout) const {
  if (length > MAX_DATA_LENGTH) {
    throw std::domain_error{"Size is too large to send via CAN"};
  }
  SendImpl(data, length, id, timeout);
}

void CanSender::Send(const void *const data, const std::size_t length,
                     const std::chrono::nanoseconds timeout) const {
  Send(data, length, default_id_, timeout);
}

void CanSender::SendFd(const void *const data, const std::size_t length,
                        const CanId id,
                        const std::chrono::nanoseconds timeout) const {
  if (length > MAX_FD_DATA_LENGTH) {
    throw std::domain_error{"Size is too large to send via CAN FD"};
  }
  SendFdImpl(data, length, id, timeout);
}

void CanSender::SendFd(const void *const data, const std::size_t length,
                        const std::chrono::nanoseconds timeout) const {
  SendFd(data, length, default_id_, timeout);
}

void CanSender::Wait(const std::chrono::nanoseconds timeout) const {
  if (decltype(timeout)::zero() < timeout) {
    auto c_timeout = ToTimeval(timeout);
    auto write_set = SingleSet(file_descriptor_);
    // Wait
    if (0 == select(file_descriptor_ + 1, NULL, &write_set, NULL, &c_timeout)) {
      throw SocketCanTimeout{"CAN Send Timeout"};
    }

    if (!FD_ISSET(file_descriptor_, &write_set)) {
      throw SocketCanTimeout{"CAN Send timeout"};
    }
  }
}

void CanSender::SendImpl(const void *const data, const std::size_t length,
                          const CanId id,
                          const std::chrono::nanoseconds timeout) const {
  if (enable_fd_) {
    throw std::runtime_error{"Tried to send standard frame from FD socket"};
  }

  // Use select call on positive timeout
  Wait(timeout);
  // Actually send the data
  constexpr int flags = 0;
  struct can_frame data_frame;
  data_frame.can_id = id.Get();
  // User facing functions do check
  data_frame.can_dlc = static_cast<decltype(data_frame.can_dlc)>(length);
  (void)std::memcpy(static_cast<void *>(&data_frame.data[0U]), data, length);
  const auto bytes_sent =
      ::send(file_descriptor_, &data_frame, sizeof(data_frame), flags);
  if (0 > bytes_sent) {
    throw std::runtime_error{strerror(errno)};
  }
}

void CanSender::SendFdImpl(const void *const data, const std::size_t length,
                             const CanId id,
                             const std::chrono::nanoseconds timeout) const {
  if (!enable_fd_) {
    throw std::runtime_error{"Tried to send FD frame from standard socket"};
  }

  // Use select call on positive timeout
  Wait(timeout);
  // Actually send the data
  constexpr int flags = 0;  // TODO(c.ho) not implemented
  struct canfd_frame data_frame;
  data_frame.can_id = id.Get();
  // User facing functions do check
  data_frame.len = static_cast<decltype(data_frame.len)>(length);
  (void)std::memcpy(static_cast<void *>(&data_frame.data[0U]), data, length);
  const auto bytes_sent =
      ::send(file_descriptor_, &data_frame, sizeof(data_frame), flags);
  if (0 > bytes_sent) {
    throw std::runtime_error{strerror(errno)};
  }
}

}  // namespace can
}  // namespace bot
