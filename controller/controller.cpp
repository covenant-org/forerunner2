#include "controller.hpp"
#include "message.hpp"
#include <capnp_schemas/generics.capnp.h>

Controller::Controller(int argc, char** argv) : Core::Vertex(argc, argv) {
  _server = this->create_action_server<Command, GenericResponse>(
      "controller", std::bind(&Controller::handle_request, this,
                              std::placeholders::_1, std::placeholders::_2));
}

void Controller::handle_request(const Core::IncomingMessage<Command>& command,
                                GenericResponse::Builder& msg) {
  msg.setCode(200);
  msg.setMessage("OK");
  if (command.content.which() == Command::LAND) {
    printf("Landing\n");
  }
}

void Controller::run() {
  while (true) sleep(1);
}

int main(int argc, char** argv) {
  auto controller = Controller(argc, argv);
  controller.run();
  return 0;
}
