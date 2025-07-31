
#ifndef COMMANDER_HPP
#define COMMANDER_HPP

#include "message.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"
#include "vertex.hpp"
#include <capnp_schemas/mavlink.capnp.h>
#include <capnp_schemas/generics.capnp.h>

class Commander : Core::Vertex {
 private:
  std::shared_ptr<Core::ActionClient<MissionCommand, GenericResponse>> _mission_client;

 public:
  Commander(Core::ArgumentParser);
  void run() override;
};

#endif
