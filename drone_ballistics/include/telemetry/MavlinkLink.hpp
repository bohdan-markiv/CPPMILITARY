#pragma once

#include <netinet/in.h>
#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <string>

namespace telemetry {

class UdpLink {
public:
  UdpLink(const std::string &host, std::uint16_t port);
  ~UdpLink();

  UdpLink(const UdpLink &) = delete;
  UdpLink &operator=(const UdpLink &) = delete;

  [[nodiscard]] bool ok() const { return fd_ >= 0; }

  bool send(const std::uint8_t *data, std::size_t len);

  // Non-blocking. Returns bytes read, 0 if nothing waiting, -1 on error.
  ssize_t receive(std::uint8_t *buf, std::size_t cap);

private:
  int fd_ = -1;
  sockaddr_in dest_{};
};

bool parse_endpoint(const std::string &text, std::string &host, std::uint16_t &port);

struct GeoPoint {
  double lat_deg = 0.0;
  double lon_deg = 0.0;
};

GeoPoint local_to_geo(double x_east_m, double y_north_m);
void send_heartbeat(UdpLink &link);
struct TelemetryState {
  double x_east_m = 0.0;
  double y_north_m = 0.0;
  double alt_m = 0.0;
  double speed_ms = 0.0;
  double direction_rad = 0.0;  // math convention: 0 = +x (East), CCW
  std::uint32_t time_boot_ms = 0;
};

void send_telemetry(UdpLink &link, const TelemetryState &s);

bool poll_for_ack(UdpLink &link, std::uint16_t command);
enum class DropStatus { Idle, Pending, Acked, Failed };

class DropCommander {
public:
  explicit DropCommander(UdpLink &link)
    : link_(link)
  {
  }

  // Call once, at the release instant.
  // Call once, at the release instant.
  void request(std::uint32_t now_ms, double lat_deg, double lon_deg, double alt_m);

  // Call every step. Handles retries and ACK polling.
  void tick(std::uint32_t now_ms);

  [[nodiscard]] DropStatus status() const { return status_; }
  [[nodiscard]] int attempts() const { return attempts_; }

private:
  void transmit(std::uint32_t now_ms);

  UdpLink &link_;
  DropStatus status_ = DropStatus::Idle;
  double lat_deg_ = 0.0;
  double lon_deg_ = 0.0;
  double alt_m_ = 0.0;
  int attempts_ = 0;
  std::uint32_t last_send_ms_ = 0;
};
}  // namespace telemetry
