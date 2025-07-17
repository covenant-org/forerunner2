#include "publisher.hpp"
#include "subscriber.hpp"
#include "vertex.hpp"
#include <capnp_schemas/controller.capnp.h>
#include <capnp_schemas/generics.capnp.h>
#include <capnp_schemas/mavlink.capnp.h>
#include <mavsdk/connection_result.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <memory>
#include <string>

#define MAVLINK_URI "udpin://0.0.0.0:14540"

// https://mavlink.io/en/messages/common.html#HOME_POSITION
#define HOME_POSITION_MESSAGE_ID 242

#ifndef MAVLINK_HPP
#define MAVLINK_HPP

class MavlinkVertex : public Core::Vertex {
 private:
  std::shared_ptr<Core::Publisher<HomePosition>> _home_position_publisher;
  std::shared_ptr<Core::ActionServer<Command, GenericResponse>> _command_action_server;

  mavsdk::Mavsdk _mavsdk;
  std::optional<std::shared_ptr<mavsdk::System>> _system = std::nullopt;
  std::shared_ptr<mavsdk::Telemetry> _telemetry;
  std::shared_ptr<mavsdk::Action> _action;
  std::shared_ptr<mavsdk::Offboard> _offboard;
  std::shared_ptr<mavsdk::MavlinkPassthrough> _passthrough;

  mavlink_home_position_t _mavlink_home_position;

  bool init_mavlink_connection(const std::string &);

 public:
  MavlinkVertex(int, char **);

  void command_cb(const Core::IncomingMessage<Command> &, GenericResponse::Builder&);

  void run();
};

#endif
