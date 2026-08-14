#include "telemetry/MavlinkLink.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <cmath>

namespace telemetry {
namespace {
constexpr double kLat0 = 50.4501;
constexpr double kLon0 = 30.5234;
constexpr double kMetersPerDegree = 111320.0;
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
// namespace

GeoPoint local_to_geo(double x_east_m, double y_north_m)
{
  GeoPoint g;
  g.lat_deg = kLat0 + (y_north_m / kMetersPerDegree);
  g.lon_deg = kLon0 + (x_east_m / (kMetersPerDegree * std::cos(kLat0 * M_PI / 180.0)));
  return g;
}
}  // namespace telemetry
