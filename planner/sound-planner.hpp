#ifndef SOUND_PLANNER_HPP
#define SOUND_PLANNER_HPP

#include "vertex.hpp"
#include <capnp_schemas/controller.capnp.h>
#include <capnp_schemas/generics.capnp.h>
#include <capnp_schemas/mavlink.capnp.h>

class SoundPlanner : public Core::Vertex {
 private:
  std::shared_ptr<Core::Subscriber<Odometry>> _odmetry_sub;
  std::shared_ptr<Core::Subscriber<Telemetry>> _telemetry_sub;
  std::shared_ptr<Core::ActionClient<Command, GenericResponse>> _command_client;
  
  int _telemetry_count;
  bool _is_armed;
  bool _is_in_air;
  bool _arm_requested;
  bool _takeoff_requested;

 public:
  explicit SoundPlanner(int argc, char** argv);
  void run();
  void odometry_cb(const Core::IncomingMessage<Odometry>&);
  void telemetry_cb(const Core::IncomingMessage<Telemetry>&);
};

#endif
