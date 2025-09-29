#include "argument_parser.hpp"
#include "capnp_schemas/generics.capnp.h"
#include "message.hpp"
#include "vertex.hpp"
#include <capnp/blob.h>
#include <iostream>
#include <memory>
#include <string>

class Chatter : public Core::Vertex {
 private:
  std::shared_ptr<Core::Subscriber<GenericResponse>> _sub;
  std::shared_ptr<Core::Publisher<GenericResponse>> _pub;

 public:
  Chatter(Core::ArgumentParser args) : Core::Vertex(args) {
    auto topic = args.get_argument("--topic");
    if (this->_args.get_argument<bool>("--talker")) {
      this->_logger.info("Starting talker");
      this->_pub = this->create_publisher<GenericResponse>(topic);
    } else {
      this->_logger.info("Starting listener");
      this->_sub = this->create_subscriber<GenericResponse>(
          topic, std::bind(&Chatter::_chatter_cb, this, std::placeholders::_1));
    }
  };

  void _chatter_cb(const Core::IncomingMessage<GenericResponse>& msg) {
    auto content = msg.content.getMessage();
    std::string messsage(content.cStr(), content.size());
    this->_logger.info("%s", messsage.c_str());
  }

  void run() {
    if (this->_args.get_argument<bool>("--listener"))
      return Core::Vertex::run();
    this->_logger.info("Start typing to the listener");
    while (true) {
      std::string input;
      std::getline(std::cin, input);
      auto msg = _pub->new_msg();
      ::capnp::Text::Reader reader(input.data(), input.size());
      msg.content.setMessage(reader);
      msg.publish();
    }
  }
};

int main(int argc, char** argv) {
  Core::BaseArgumentParser args(argc, argv);
  auto& group = args.add_mutually_exclusive_group(true);
  group.add_argument("--talker")
      .implicit_value(true)
      .default_value(false)
      .flag();
  group.add_argument("--listener")
      .implicit_value(true)
      .default_value(false)
      .flag();
  args.add_argument("--topic").default_value("chatter");
  Chatter chatter(args);
  chatter.run();
}
