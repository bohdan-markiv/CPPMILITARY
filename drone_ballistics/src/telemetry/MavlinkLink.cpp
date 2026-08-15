#include "telemetry/MavlinkLink.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <cmath>
#include <common/mavlink.h>

namespace telemetry {
namespace {
constexpr double kLat0 = 50.4501;
constexpr double kLon0 = 30.5234;
constexpr double kMetersPerDegree = 111320.0;

constexpr std::uint8_t kSysId = 1;
constexpr std::uint8_t kCompId = MAV_COMP_ID_AUTOPILOT1;

bool send_message(UdpLink &link, const mavlink_message_t &msg)
{
  std::uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const std::uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  return link.send(buf, len);
}
}  // namespace
bool parse_endpoint(const std::string &text, std::string &host, std::uint16_t &port)
{
  const auto colon = text.rfind(':');
  if (colon == std::string::npos || colon == 0 || colon + 1 == text.size()) {
    return false;
  }
  try {
    const int value = std::stoi(text.substr(colon + 1));
    if (value <= 0 || value > 65535) {
      return false;
    }
    port = static_cast<std::uint16_t>(value);
  }
  catch (const std::exception &) {
    return false;
  }
  host = text.substr(0, colon);
  return true;
}

UdpLink::UdpLink(const std::string &host, std::uint16_t port)
{
  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) {
    std::perror("socket");
    return;
  }

  std::memset(&dest_, 0, sizeof(dest_));
  dest_.sin_family = AF_INET;
  dest_.sin_port = htons(port);

  if (::inet_pton(AF_INET, host.c_str(), &dest_.sin_addr) != 1) {
    std::fprintf(stderr, "bad destination address: %s\n", host.c_str());
    ::close(fd_);
    fd_ = -1;
  }
}

UdpLink::~UdpLink()
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

bool UdpLink::send(const std::uint8_t *data, std::size_t len)
{
  if (fd_ < 0) {
    return false;
  }

  const ssize_t sent = ::sendto(fd_, data, len, 0, reinterpret_cast<const sockaddr *>(&dest_), sizeof(dest_));
  if (sent < 0) {
    std::perror("sendto");
    return false;
  }
  return static_cast<std::size_t>(sent) == len;
}

ssize_t UdpLink::receive(std::uint8_t *buf, std::size_t cap)
{
  if (fd_ < 0) {
    return -1;
  }

  const ssize_t n = ::recvfrom(fd_, buf, cap, MSG_DONTWAIT, nullptr, nullptr);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return 0;  // nothing waiting is normal, not an error
    }
    std::perror("recvfrom");
    return -1;
  }
  return n;
}

GeoPoint local_to_geo(double x_east_m, double y_north_m)
{
  GeoPoint g;
  g.lat_deg = kLat0 + (y_north_m / kMetersPerDegree);
  g.lon_deg = kLon0 + (x_east_m / (kMetersPerDegree * std::cos(kLat0 * M_PI / 180.0)));
  return g;
}

void send_heartbeat(UdpLink &link)
{
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(
    kSysId, kCompId, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_FLAG_SAFETY_ARMED, 0, MAV_STATE_ACTIVE);
  send_message(link, msg);
}
void send_telemetry(UdpLink &link, const TelemetryState &s)
{
  const GeoPoint geo = local_to_geo(s.x_east_m, s.y_north_m);

  // Your `direction` is CCW from East. MAVLink wants compass: CW from North.
  double heading_rad = M_PI / 2.0 - s.direction_rad;
  while (heading_rad < 0.0)
    heading_rad += 2.0 * M_PI;
  while (heading_rad >= 2.0 * M_PI)
    heading_rad -= 2.0 * M_PI;

  // Velocity components in the local frame, then mapped to MAVLink's NED.
  const double v_east = s.speed_ms * std::cos(s.direction_rad);
  const double v_north = s.speed_ms * std::sin(s.direction_rad);

  mavlink_message_t msg;

  mavlink_msg_global_position_int_pack(kSysId,
                                       kCompId,
                                       &msg,
                                       s.time_boot_ms,
                                       static_cast<std::int32_t>(std::llround(geo.lat_deg * 1e7)),
                                       static_cast<std::int32_t>(std::llround(geo.lon_deg * 1e7)),
                                       static_cast<std::int32_t>(std::llround(s.alt_m * 1000.0)),  // alt, mm
                                       static_cast<std::int32_t>(std::llround(s.alt_m * 1000.0)),  // relative_alt, mm
                                       static_cast<std::int16_t>(std::llround(v_north * 100.0)),   // vx = NORTH, cm/s
                                       static_cast<std::int16_t>(std::llround(v_east * 100.0)),    // vy = EAST, cm/s
                                       0,                                                          // vz = DOWN, cm/s
                                       static_cast<std::uint16_t>(std::llround(heading_rad * 180.0 / M_PI * 100.0)));
  send_message(link, msg);

  mavlink_msg_attitude_pack(kSysId,
                            kCompId,
                            &msg,
                            s.time_boot_ms,
                            0.0F,
                            0.0F,                             // roll, pitch — not modelled
                            static_cast<float>(heading_rad),  // yaw, radians
                            0.0F,
                            0.0F,
                            0.0F);  // body rates
  send_message(link, msg);
}
bool poll_for_ack(UdpLink &link, std::uint16_t command)
{
  std::uint8_t buf[2048];
  bool accepted = false;

  for (;;) {
    const ssize_t n = link.receive(buf, sizeof(buf));
    if (n <= 0) {
      break;  // nothing left waiting
    }

    mavlink_message_t msg;
    mavlink_status_t status;
    for (ssize_t i = 0; i < n; ++i) {
      if (mavlink_parse_char(MAVLINK_COMM_1, buf[i], &msg, &status) != 1) {
        continue;
      }
      if (msg.msgid != MAVLINK_MSG_ID_COMMAND_ACK) {
        continue;
      }

      mavlink_command_ack_t ack;
      mavlink_msg_command_ack_decode(&msg, &ack);
      if (ack.command == command && ack.result == MAV_RESULT_ACCEPTED) {
        accepted = true;
      }
    }
  }
  return accepted;
}
namespace {
constexpr std::uint32_t kAckTimeoutMs = 500;
constexpr int kMaxAttempts = 5;
}  // namespace

void DropCommander::transmit(std::uint32_t now_ms)
{
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(kSysId,
                                kCompId,
                                &msg,
                                kSysId,
                                kCompId,  // target system/component
                                MAV_CMD_USER_1,
                                static_cast<std::uint8_t>(attempts_),  // confirmation counter
                                0.0F,
                                0.0F,
                                0.0F,
                                0.0F,                          // param1..4 unused
                                static_cast<float>(lat_deg_),  // param5 = lat
                                static_cast<float>(lon_deg_),  // param6 = lon
                                static_cast<float>(alt_m_));   // param7 = alt, metres
  send_message(link_, msg);

  ++attempts_;
  last_send_ms_ = now_ms;
  std::printf("[drop] COMMAND_LONG attempt %d/%d\n", attempts_, kMaxAttempts);
}

// Call once, at the release instant.
void DropCommander::request(std::uint32_t now_ms, double lat_deg, double lon_deg, double alt_m)
{
  if (status_ != DropStatus::Idle) {
    return;  // only one drop per flight
  }
  status_ = DropStatus::Pending;
  lat_deg_ = lat_deg;
  lon_deg_ = lon_deg;
  alt_m_ = alt_m;
  attempts_ = 0;
  transmit(now_ms);
}

void DropCommander::tick(std::uint32_t now_ms)
{
  if (poll_for_ack(link_, MAV_CMD_USER_1) && status_ == DropStatus::Pending) {
    status_ = DropStatus::Acked;
    std::printf("[drop] ACK received after %d attempt(s)\n", attempts_);
    return;
  }

  if (status_ != DropStatus::Pending) {
    return;  // silence after ACK, and nothing to do before the drop
  }

  if (now_ms - last_send_ms_ >= kAckTimeoutMs) {
    if (attempts_ >= kMaxAttempts) {
      status_ = DropStatus::Failed;
      std::printf("[drop] ACK not received after %d attempts\n", kMaxAttempts);
    }
    else {
      transmit(now_ms);
    }
  }
}
}  // namespace telemetry