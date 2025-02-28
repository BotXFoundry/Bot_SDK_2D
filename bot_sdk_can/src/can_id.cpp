#include "bot_sdk_can/can_id.hpp"

#include <linux/can.h>

#include <utility>

namespace bot {
namespace can {

static_assert(MAX_DATA_LENGTH == sizeof(std::declval<struct can_frame>().data),
              "Unexpected CAN frame data size");
static_assert(MAX_FD_DATA_LENGTH ==
                  sizeof(std::declval<struct canfd_frame>().data),
              "Unexpected CAN FD frame data size");
static_assert(std::is_same<uint32_t, canid_t>::value,
              "Underlying type of CanId is incorrect");
constexpr uint32_t EXTENDED_MASK = CAN_EFF_FLAG;
constexpr uint32_t REMOTE_MASK = CAN_RTR_FLAG;
constexpr uint32_t ERROR_MASK = CAN_ERR_FLAG;
constexpr uint32_t EXTENDED_ID_MASK = CAN_EFF_MASK;
constexpr uint32_t STANDARD_ID_MASK = CAN_SFF_MASK;

CanId::CanId(const uint32_t raw_id, const uint64_t bus_time,
             const uint32_t data_length)
    : id_{raw_id}, data_length_{data_length}, bus_time_(bus_time) {
  (void)GetFrameType();  // just to throw
}
CanId::CanId(const uint32_t id, const uint64_t bus_time, FrameType type,
             StandardFrame_)
    : CanId{id, bus_time, type, false} {}

CanId::CanId(const uint32_t id, const uint64_t bus_time, FrameType type,
             ExtendedFrame_)
    : CanId{id, bus_time, type, true} {}

CanId::CanId(const uint32_t id, const uint64_t bus_time, FrameType type,
             bool is_extended)
    : bus_time_(bus_time) {
  // Set extended bit
  if (is_extended) {
    (void)Extended();
  }
  (void)SetFrameType(type);
  (void)SetId(id);
}

CanId& CanId::Standard() noexcept {
  id_ = id_ & (~EXTENDED_MASK);
  return *this;
}

CanId& CanId::Extended() noexcept {
  id_ = id_ | EXTENDED_MASK;
  return *this;
}

CanId& CanId::ErrorFrame() noexcept {
  id_ = id_ & (~REMOTE_MASK);
  id_ = id_ | ERROR_MASK;
  return *this;
}

CanId& CanId::RemoteFrame() noexcept {
  id_ = id_ & (~ERROR_MASK);
  id_ = id_ | REMOTE_MASK;
  return *this;
}

CanId& CanId::DataFrame() noexcept {
  id_ = id_ & (~ERROR_MASK);
  id_ = id_ & (~REMOTE_MASK);
  return *this;
}

CanId& CanId::SetFrameType(const FrameType type) {
  switch (type) {
    case FrameType::DATA:
      (void)DataFrame();
      break;
    case FrameType::ERROR:
      (void)ErrorFrame();
      break;
    case FrameType::REMOTE:
      (void)RemoteFrame();
      break;
    default:
      throw std::logic_error{"CanId: No such type"};
  }
  return *this;
}

CanId& CanId::SetId(const uint32_t id) {
  // Can specification: http://esd.cs.ucr.edu/webres/can20.pdf
  // says "The 7 most significant bits cannot all be recessive (value of 1)", pg
  // 11
  constexpr auto MAX_EXTENDED = 0x1FBF'FFFFU;
  constexpr auto MAX_STANDARD = 0x07EFU;
  static_assert(MAX_EXTENDED <= EXTENDED_ID_MASK,
                "Max extended id value is wrong");
  static_assert(MAX_STANDARD <= STANDARD_ID_MASK,
                "Max extended id value is wrong");
  const auto max_id = IsExtended() ? MAX_EXTENDED : MAX_STANDARD;
  if (max_id < id) {
    throw std::domain_error{"CanId would be truncated!"};
  }
  // Clear and set

  id_ =
      id_ & (~EXTENDED_ID_MASK);  // clear ALL ID bits, not just standard bits
  id_ = id_ | id;
  return *this;
}

uint32_t CanId::Get() const noexcept { return id_; }

bool CanId::IsExtended() const noexcept {
  return (id_ & EXTENDED_MASK) == EXTENDED_MASK;
}

uint32_t CanId::GetId() const noexcept {
  const auto mask = IsExtended() ? EXTENDED_ID_MASK : STANDARD_ID_MASK;
  return id_ & mask;
}

uint32_t CanId::Length() const noexcept { return data_length_; }

FrameType CanId::GetFrameType() const {
  const auto is_error = (id_ & ERROR_MASK) == ERROR_MASK;
  const auto is_remote = (id_ & REMOTE_MASK) == REMOTE_MASK;
  if (is_error && is_remote) {
    throw std::domain_error{"CanId has both bits 29 and 30 set! Inconsistent!"};
  }

  if (is_error) {
    return FrameType::ERROR;
  }
  if (is_remote) {
    return FrameType::REMOTE;
  }
  return FrameType::DATA;
}

}  // namespace can
}  // namespace bot
