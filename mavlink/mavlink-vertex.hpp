#include "capnp_schemas/mavlink.capnp.h"
#include "publisher.hpp"
#include "subscriber.hpp"
#include "vertex.hpp"
#include <mavsdk/connection_result.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <memory>
#include <string>

#define MAVLINK_URI "udpin://0.0.0.0:14540"

#ifndef MAVLINK_HPP
#define MAVLINK_HPP

class MavlinkVertex : public Core::Vertex {
 private:
  std::shared_ptr<Core::Publisher<Telemetry>> _telemetry_publisher;
  std::shared_ptr<Core::Subscriber<Takeoff>> _takeoff_subscriber;
  std::shared_ptr<Core::Subscriber<Land>> _land_subscriber;

  mavsdk::Mavsdk _mavsdk;
  std::optional<std::shared_ptr<mavsdk::System>> _system = std::nullopt;
  std::shared_ptr<mavsdk::Telemetry> _telemetry;
  std::shared_ptr<mavsdk::Action> _action;
  std::shared_ptr<mavsdk::Offboard> _offboard;

  bool init_mavlink_connection(const std::string &);

 public:
  MavlinkVertex(int, char **);

  void takeoff_cb(const Core::IncomingMessage<Takeoff> &);
  void land_cb(const Core::IncomingMessage<Land> &);

  void run();
};

#endif
