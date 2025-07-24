#ifndef SOUND_PLANNER_HPP
#define SOUND_PLANNER_HPP

#include "vertex.hpp"
#include <capnp_schemas/controller.capnp.h>
#include <capnp_schemas/generics.capnp.h>
#include <capnp_schemas/mavlink.capnp.h>

class SoundPlanner : public Core::Vertex {
 private:
  std::shared_ptr<Core::Subscriber<Odometry>> _odmetry_sub;
  std::shared_ptr<Core::ActionClient<Command, GenericResponse>> _command_client;

 public:
  explicit SoundPlanner(int argc, char** argv);
  void run();
  void odometry_cb(const Core::IncomingMessage<Odometry>&);
};

#endif
