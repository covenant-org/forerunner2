#include "argument_parser.hpp"
#include "message.hpp"
#include "vertex.hpp"
#include <capnp_schemas/controller.capnp.h>
#include <capnp_schemas/generics.capnp.h>
#include <capnp_schemas/mavlink.capnp.h>

#ifndef MAVLINK_BASE_ACTIONS_HPP
#define MAVLINK_BASE_ACTIONS_HPP

class MavlinkMission : public Core::Vertex {
 private:
  std::shared_ptr<Core::ActionClient<Command, GenericResponse>> _command_client;
  std::shared_ptr<Core::ActionServer<MissionCommand, GenericResponse>>
      _mission_server;
  std::shared_ptr<Core::Subscriber<Altitude>> _alt_sub;
  std::shared_ptr<Core::Subscriber<Telemetry>> _tel_sub;
  bool _is_in_air = false;
  bool _is_armed = false;
  bool _arm_requested = false;
  bool _takeoff_requested = false;
  bool _land_requested = false;
  size_t _telemetry_count = false;
  float _altitude = 0.0f;

 public:
  MavlinkMission(Core::ArgumentParser);

  void altitude_cb(const Core::IncomingMessage<Altitude> &);
  void telemetry_cb(const Core::IncomingMessage<Telemetry> &);
  void arm_and_takeoff(GenericResponse::Builder &res, float = 2.0f);
  void land(GenericResponse::Builder &res);
  void mission_command_cb(const Core::IncomingMessage<MissionCommand> &,
                          GenericResponse::Builder &);
  void run() override;
};

#endif  // MAVLINK_BASE_ACTIONS_HPP
