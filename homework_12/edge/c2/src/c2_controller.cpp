#include "c2_controller.hpp"
#include <sys/types.h>
#include "fc_link.hpp"     // MAVSDK обгортка, API описано у fc_link.hpp
#include "udp_socket.hpp"  // UDP прийом, API описано у udp_socket.hpp

#include <nlohmann/json.hpp>  // Розбiр JSON з точками маршруту вiд auto_stub

#include <fstream>
#include <iostream>
#include <string>

static constexpr uint16_t STUB_PORT = 14560;

namespace {
const char* to_string(C2State state)
{
  switch (state) {
    case C2State::DISARMED:
      return "DISARMED";
    case C2State::ARMED_HOLD:
      return "ARMED_HOLD";
    case C2State::ARMED_GUIDED:
      return "ARMED_GUIDED";
    case C2State::ARMED_MANUAL:
      return "ARMED_MANUAL";
  }
  return "UNKNOWN";
}
}  // namespace

struct C2Controller::Impl {
  explicit Impl(uint16_t fc_port)
    : fc(fc_port)
    , stub(STUB_PORT)
    , log("/var/log/c2/c2.log", std::ios::app)
  {
  }

  C2State state = C2State::DISARMED;
  FcLink fc;
  UdpSocket stub;
  std::ofstream log;

  bool healthy_written = false;
  bool hold_sent = false;

  // TODO: додати FcLink, UdpSocket, лог-файл та прапорцi стану.
  // FcLink потребує fc_port у конструкторi Impl.
  // UdpSocket має слухати STUB_PORT.

  void transition(C2State next)
  {
    // TODO: якщо next != state, записати "PREV -> NEW" у stdout i лог,
    // потiм оновити state. Якщо стан не змiнився, нiчого не писати.
    if (next != state) {
      std::cout << "[C2] state: " << to_string(state) << " -> " << to_string(next) << std::endl;
      log << "[C2] state: " << to_string(state) << " -> " << to_string(next) << std::endl;
    }
    if (next == C2State::ARMED_HOLD) {
      hold_sent = false;
    }
    state = next;
  }
};

C2Controller::C2Controller(uint16_t fc_port)
  : impl_(std::make_unique<Impl>(fc_port))
{
  // TODO: передати fc_port в Impl та вiдкрити /var/log/c2/c2.log.
}

C2Controller::~C2Controller() = default;

void C2Controller::tick()
{
  if (impl_->healthy_written == false && impl_->fc.is_connected()) {
    std::ofstream("/tmp/c2_healthy").close();
    impl_->log << "[C2] healthy" << std::endl;
    impl_->healthy_written = true;
  }

  C2State next_state;
  if (!impl_->fc.is_armed()) {
    next_state = C2State::DISARMED;
  }
  else {
    switch (impl_->fc.flight_mode()) {
      case FcLink::FlightMode::Hold:
        next_state = C2State::ARMED_HOLD;
        break;
      case FcLink::FlightMode::Guided:
        next_state = C2State::ARMED_GUIDED;
        break;
      case FcLink::FlightMode::Manual:
        next_state = C2State::ARMED_MANUAL;
        break;
      default:
        next_state = C2State::ARMED_HOLD;
        break;
    }
  }

  impl_->transition(next_state);

  if (impl_->state == C2State::ARMED_HOLD && !impl_->hold_sent) {
    impl_->fc.hold();
    impl_->hold_sent = true;
  }

  char buf[200];
  ssize_t n = impl_->stub.recv(buf, sizeof(buf));
  if (n <= 0) {
    // std::cout << "[C2] No data received from auto_stub" << std::endl;
    // impl_->log << "[C2] No data received from auto_stub" << std::endl;
    return;  // немає даних
  }
  else if (impl_->state == C2State::ARMED_GUIDED) {
    try {
      nlohmann::json j = nlohmann::json::parse(buf, buf + n);
      float north = j["north_m"];
      float east = j["east_m"];
      impl_->fc.go_to_ned(north, east);
      std::cout << "[C2] fwd: north=" << north << " east=" << east << std::endl;
      impl_->log << "[C2] fwd: north=" << north << " east=" << east << std::endl;
    }
    catch (const std::exception& e) {
      std::cout << "[C2] JSON parse error: " << e.what() << std::endl;
      impl_->log << "[C2] JSON parse error: " << e.what() << std::endl;
    }
  }
  else {
    std::cout << "[C2] blocked: waypoint in " << to_string(impl_->state) << std::endl;
    impl_->log << "[C2] blocked: waypoint in " << to_string(impl_->state) << std::endl;
  }
}
C2State C2Controller::current_state() const
{
  return impl_->state;
}