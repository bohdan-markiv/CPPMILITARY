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
}  // namespace telemetry
