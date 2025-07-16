#include "mavlink-vertex.hpp"
#include <iostream>
#include <mavsdk/connection_result.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;

MavlinkVertex::MavlinkVertex(int argc, char **argv)
    : Core::Vertex(argc, argv),
      _mavsdk(Mavsdk::Configuration{ComponentType::GroundStation}) {
  auto result = this->init_mavlink_connection(MAVLINK_URI);
  if (!result) {
    throw std::runtime_error("error initializing mavsdk");
  }

  this->_takeoff_subscriber = this->create_subscriber<Takeoff>(
      "takeoff",
      std::bind(&MavlinkVertex::takeoff_cb, this, std::placeholders::_1));

  this->_land_subscriber = this->create_subscriber<Land>(
      "land", std::bind(&MavlinkVertex::land_cb, this, std::placeholders::_1));
}

void MavlinkVertex::land_cb(const Core::IncomingMessage<Land> &) {
  const auto land_result = this->_action->land();
  if (land_result != mavsdk::Action::Result::Success) {
    throw std::runtime_error("error landing");
  }
}

void MavlinkVertex::takeoff_cb(const Core::IncomingMessage<Takeoff> &msg) {
  const auto arm_result = this->_action->arm();
  if (arm_result != mavsdk::Action::Result::Success) {
    throw std::runtime_error("failed to arm");
  }
  const auto set_takeoff_altitude_result =
      this->_action->set_takeoff_altitude(msg.content.getAltitude());
  if (set_takeoff_altitude_result != mavsdk::Action::Result::Success) {
    throw std::runtime_error("failed to set takeoff altitude");
  }
  const auto takeoff_result = this->_action->takeoff();
  if (takeoff_result != mavsdk::Action::Result::Success) {
    throw std::runtime_error("failed to takeoff");
  }
}

bool MavlinkVertex::init_mavlink_connection(const std::string &uri) {
  ConnectionResult result = this->_mavsdk.add_any_connection(uri);
  if (result != ConnectionResult::Success) {
    return false;
  }

  this->_system = this->_mavsdk.first_autopilot(3.0);
  if (!this->_system) {
    return false;
  }

  this->_telemetry = std::make_shared<mavsdk::Telemetry>(this->_system.value());
  this->_action = std::make_shared<mavsdk::Action>(this->_system.value());
  this->_offboard = std::make_shared<mavsdk::Offboard>(this->_system.value());

  return true;
}

void MavlinkVertex::run() {
  std::cout << "hello world" << std::endl;
  while (true) {
    sleep(1);
  }
}

int main(int argc, char **argv) {
  std::shared_ptr<MavlinkVertex> mavlink =
      std::make_shared<MavlinkVertex>(argc, argv);
  mavlink->run();
  return 0;
}
