
#ifndef COMMANDER_HPP
#define COMMANDER_HPP

#include "message.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"
#include "vertex.hpp"
#include <Eigen/Dense>
#include <capnp_schemas/mavlink.capnp.h>

class Commander : Core::Vertex {
 private:
  std::shared_ptr<Core::ActionClient<Comand, GenericResponse>> _action_client;

 public:
  Commander(int, char**);

  void run();
};

#endif
