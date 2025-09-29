
#include "argument_parser.hpp"
#include "capnp_schemas/generics.capnp.h"
#include "message.hpp"
#include "vertex.hpp"
#include <capnp/blob.h>
#include <iostream>
#include <memory>
#include <string>

class ActionTester : public Core::Vertex {
 private:
  std::shared_ptr<Core::ActionServer<KeyValue, GenericResponse>> _server;
  std::shared_ptr<Core::ActionClient<KeyValue, GenericResponse>> _client;

 public:
  ActionTester(Core::ArgumentParser args) : Core::Vertex(args) {
    auto topic = args.get_argument("--topic");
    if (this->_args.get_argument<bool>("--server")) {
      this->_logger.info("Starting server");
      this->_server = this->create_action_server<KeyValue, GenericResponse>(
          topic, std::bind(&ActionTester::_cb, this, std::placeholders::_1));
    } else {
      this->_logger.info("Starting client");
      this->_client =
          this->create_action_client<KeyValue, GenericResponse>(topic);
    }
  };

  void _cb(const Core::IncomingMessage<KeyValue>& msg) {
    this->_logger.info("Working %s", msg.content.getValue().cStr());
    sleep(2);
    this->_logger.info("Done %s", msg.content.getValue().cStr());
  }

  void run() {
    if (this->_args.get_argument<bool>("--server")) return Core::Vertex::run();
    this->_logger.info("Start typing to the server");
    while (true) {
      std::string input;
      std::getline(std::cin, input);
      auto msg = _client->new_msg();
      ::capnp::Text::Reader reader(input.data(), input.size());
      msg.content.setValue(reader);
      msg.send();
      this->_logger.info("Done %s", msg.content.getValue().cStr());
    }
  }
};

int main(int argc, char** argv) {
  Core::BaseArgumentParser args(argc, argv);
  auto& group = args.add_mutually_exclusive_group(true);
  group.add_argument("--server")
      .implicit_value(true)
      .default_value(false)
      .flag();
  group.add_argument("--client")
      .implicit_value(true)
      .default_value(false)
      .flag();
  args.add_argument("--topic").default_value("test/action");
  ActionTester chatter(args);
  chatter.run();
}
