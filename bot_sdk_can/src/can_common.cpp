#include "bot_sdk_can/can_common.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace bot {
namespace can {


int32_t BindCanSocket(const std::string &interface, bool enable_fd) {
  if (interface.length() >= static_cast<std::string::size_type>(IFNAMSIZ)) {
    throw std::domain_error{"CAN interface name too long"};
  }

  // Create file descriptor
  const auto file_descriptor =
      socket(PF_CAN, static_cast<int32_t>(SOCK_RAW), CAN_RAW);
  if (0 > file_descriptor) {
    throw std::runtime_error{"Failed to open CAN socket"};
  }

  if (0 != fcntl(file_descriptor, F_SETFL, O_NONBLOCK)) {
    throw std::runtime_error{"Failed to set CAN socket to nonblocking"};
  }

  // Set up address/interface name
  struct ifreq ifr;
  // The destination struct is local; don't need address
  (void)strncpy(&ifr.ifr_name[0U], interface.c_str(), interface.length() + 1U);
  if (0 != ioctl(file_descriptor, static_cast<uint32_t>(SIOCGIFINDEX), &ifr)) {
    throw std::runtime_error{"Failed to set CAN socket name via ioctl()"};
  }

  struct sockaddr_can addr;
  addr.can_family = static_cast<decltype(addr.can_family)>(AF_CAN);
  addr.can_ifindex = ifr.ifr_ifindex;

  if (0 > bind(file_descriptor, reinterpret_cast<struct sockaddr *>(&addr),
               sizeof(addr))) {
    throw std::runtime_error{"Failed to bind CAN socket"};
  }


  // Enable CAN FD support
  const int32_t enable_canfd = enable_fd ? 1 : 0;
  if (0 != setsockopt(file_descriptor, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                      &enable_canfd, sizeof(enable_canfd))) {
    throw std::runtime_error{"Failed to enable CAN FD support"};
  }

  return file_descriptor;
}


void SetCanFilter(int32_t fd, const std::vector<struct can_filter> &f_list) {
  if (0 != setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER,
                      f_list.empty() ? NULL : f_list.data(),
                      sizeof(can_filter) * f_list.size())) {
    throw std::runtime_error{"Failed to set up CAN filters: " +
                             std::string{strerror(errno)}};
  }
}


void SetCanErrorFilter(int32_t fd, can_err_mask_t err_mask) {
  if (0 != setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask,
                      sizeof(err_mask))) {
    throw std::runtime_error{"Failed to set up CAN error filters: " +
                             std::string{strerror(errno)}};
  }
}


void SetCanFilterJoin(int32_t fd, bool join_filters) {
  auto join = static_cast<int>(join_filters);
  if (0 !=
      setsockopt(fd, SOL_CAN_RAW, CAN_RAW_JOIN_FILTERS, &join, sizeof(join))) {
    throw std::runtime_error{"Failed to set up joined CAN filters: " +
                             std::string{strerror(errno)}};
  }
}


struct timeval ToTimeval(const std::chrono::nanoseconds timeout) noexcept {
  const auto count = timeout.count();
  constexpr auto BILLION = 1'000'000'000LL;
  struct timeval c_timeout;
  c_timeout.tv_sec = static_cast<decltype(c_timeout.tv_sec)>(count / BILLION);
  c_timeout.tv_usec =
      static_cast<decltype(c_timeout.tv_usec)>((count % BILLION) / 1000LL);

  return c_timeout;
}


uint64_t FromTimeval(const struct timeval tv) noexcept {
  return static_cast<uint64_t>(tv.tv_sec) * 1e6 + tv.tv_usec;
}


fd_set SingleSet(int32_t file_descriptor) noexcept {
  fd_set descriptor_set;
  FD_ZERO(&descriptor_set);
  FD_SET(file_descriptor, &descriptor_set);

  return descriptor_set;
}
}  // namespace can
}  // namespace bot
