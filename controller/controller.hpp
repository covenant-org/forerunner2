#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "capnp_schemas/controller.capnp.h"
#include "capnp_schemas/generics.capnp.h"
#include "message.hpp"
#include "vertex.hpp"

class Controller : public Core::Vertex {
 private:
  std::shared_ptr<Core::ActionServer<Command, GenericResponse>> _server;

 public:
  Controller(int argc, char** argv);
  void handle_request(const Core::IncomingMessage<Command>& req,
                      GenericResponse::Builder& res);

  void run();
};

#endif  // !CONTROLLER_HPP
