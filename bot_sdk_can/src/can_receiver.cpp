#include "bot_sdk_can/can_receiver.hpp"

#include <linux/can.h>
#include <linux/sockios.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "bot_sdk_can/can_common.hpp"

namespace bot {
namespace can {

CanReceiver::CanReceiver(const std::string &interface, const bool enable_fd)
    : file_descriptor_{BindCanSocket(interface, enable_fd)},
      enable_fd_(enable_fd) {}

CanReceiver::~CanReceiver() noexcept {
  // Can't do anything on error; in fact generally shouldn't on close() error
  (void)close(file_descriptor_);
}

CanReceiver::CanFilterList::CanFilterList(const char *str) {
  *this = ParseFilters(str);
}

CanReceiver::CanFilterList::CanFilterList(const std::string &str) {
  *this = ParseFilters(str);
}

CanReceiver::CanFilterList CanReceiver::CanFilterList::ParseFilters(
    const std::string &str) {
  CanFilterList filter_list;
  filter_list.error_mask = 0;
  filter_list.join_filters = false;

  std::istringstream input(str);
  std::string fstr;

  while (getline(input, fstr, ',')) {
    // trim leading and trailing whitespaces
    fstr = fstr.substr(
        fstr.find_first_not_of(" \t"),
        fstr.find_last_not_of(" \t") - fstr.find_first_not_of(" \t") + 1);

    struct can_filter filter;
    if (std::sscanf(fstr.c_str(), "%x:%x", &filter.can_id, &filter.can_mask) ==
        2) {
      filter.can_mask &= ~CAN_ERR_FLAG;
      if (fstr.size() > 8 && fstr[8] == ':') {
        filter.can_id |= CAN_EFF_FLAG;
      }
      filter_list.filters.push_back(filter);
    } else if (std::sscanf(fstr.c_str(), "%x~%x", &filter.can_id,
                           &filter.can_mask) == 2) {
      filter.can_id |= CAN_INV_FILTER;
      filter.can_mask &= ~CAN_ERR_FLAG;
      if (fstr.size() > 8 && fstr[8] == '~') {
        filter.can_id |= CAN_EFF_FLAG;
      }
      filter_list.filters.push_back(filter);
    } else if (fstr == "j" || fstr == "J") {
      filter_list.join_filters = true;
    } else if (std::sscanf(fstr.c_str(), "#%x", &filter_list.error_mask) != 1) {
      throw std::runtime_error("Error during filter parsing: " + fstr);
    }
  }
  return filter_list;
}

void CanReceiver::SetCanFilters(const CanFilterList &filters) {
  SetCanFilter(file_descriptor_, filters.filters);
  SetCanErrorFilter(file_descriptor_, filters.error_mask);
  SetCanFilterJoin(file_descriptor_, filters.join_filters);
}

void CanReceiver::Wait(const std::chrono::nanoseconds timeout) const {
  if (decltype(timeout)::zero() < timeout) {
    auto c_timeout = ToTimeval(timeout);
    auto read_set = SingleSet(file_descriptor_);
    // Wait
    if (0 == select(file_descriptor_ + 1, &read_set, NULL, NULL, &c_timeout)) {
      throw SocketCanTimeout{"CAN Receive Timeout"};
    }
    if (!FD_ISSET(file_descriptor_, &read_set)) {
      throw SocketCanTimeout{"CAN Receive timeout"};
    }
  }
}

CanId CanReceiver::Receive(void *const data,
                           const std::chrono::nanoseconds timeout) const {
  if (enable_fd_) {
    throw std::runtime_error{"attempted to read standard frame from FD socket"};
  }

  Wait(timeout);
  // Read
  struct can_frame frame;
  const auto nbytes = read(file_descriptor_, &frame, sizeof(frame));

  // Checks
  if (nbytes < 0) {
    throw std::runtime_error{strerror(errno)};
  }
  if (static_cast<std::size_t>(nbytes) < sizeof(frame)) {
    throw std::runtime_error{"read: incomplete CAN frame"};
  }
  if (static_cast<std::size_t>(nbytes) != sizeof(frame)) {
    throw std::logic_error{"Message was wrong size"};
  }
  // Write
  const auto data_length = static_cast<uint32_t>(frame.can_dlc);
  (void)std::memcpy(data, static_cast<void *>(&frame.data[0U]), data_length);

  // get bus timestamp
  struct timeval tv;
  ioctl(file_descriptor_, SIOCGSTAMP, &tv);
  uint64_t bus_time = FromTimeval(tv);

  return CanId{frame.can_id, bus_time, data_length};
}

CanId CanReceiver::ReceiveFd(void *const data,
                              const std::chrono::nanoseconds timeout) const {
  if (!enable_fd_) {
    throw std::runtime_error{"attempted to read FD frame from standard socket"};
  }

  Wait(timeout);
  // Read
  struct canfd_frame frame;
  const auto nbytes = read(file_descriptor_, &frame, sizeof(frame));
  // Checks
  if (nbytes < 0) {
    throw std::runtime_error{strerror(errno)};
  }
  if (static_cast<std::size_t>(nbytes) < sizeof(frame)) {
    throw std::runtime_error{"read: incomplete CAN FD frame"};
  }
  if (static_cast<std::size_t>(nbytes) != sizeof(frame)) {
    throw std::logic_error{"Message was wrong size"};
  }
  // Write
  const auto data_length = static_cast<uint32_t>(frame.len);
  (void)std::memcpy(data, static_cast<void *>(&frame.data[0U]), data_length);

  // get bus timestamp
  struct timeval tv;
  ioctl(file_descriptor_, SIOCGSTAMP, &tv);
  uint64_t bus_time = FromTimeval(tv);

  return CanId{frame.can_id, bus_time, data_length};
}

}  // namespace can
}  // namespace bot
