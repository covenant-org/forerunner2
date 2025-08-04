#include "argument_parser.hpp"
#include "publisher.hpp"
#include "vertex.hpp"
#include <capnp_schemas/generics.capnp.h>
#include <capnp_schemas/mavlink.capnp.h>

class InputGoal : public Core::Vertex {
 private:
  std::shared_ptr<Core::ActionClient<Position, GenericResponse>>
      _input_action_client;

 public:
  InputGoal(Core::ArgumentParser parser) : Core::Vertex(parser) {
    this->_input_action_client =
        this->create_action_client<Position, GenericResponse>("input_goal");

    float x = this->get_argument<float>("--x");
    float y = this->get_argument<float>("--y");
    float z = this->get_argument<float>("--z");

    auto req = this->_input_action_client->new_msg();
    req.content.setX(x);
    req.content.setY(y);
    req.content.setZ(z);
    auto result = req.send();
    auto response = result.value().content;
    if (response.getCode() > 300) {
      this->_logger.error("failed to send goal with code %d and message: %s",
                          response.getCode(), response.getMessage());
    } else {
      this->_logger.info("success sending goal");
    }
  }

  void run() override {}
};

int main(int argc, char** argv) {
  Core::BaseArgumentParser parser(argc, argv);
  parser.add_argument("--x")
      .help("x in local coordinates")
      .required()
      .scan<'g', float>();
  parser.add_argument("--y")
      .help("y in local coordinates")
      .required()
      .scan<'g', float>();
  parser.add_argument("--z")
      .help("z in local coordinates")
      .required()
      .scan<'g', float>();

  std::shared_ptr<InputGoal> vertex = std::make_shared<InputGoal>(parser);
  vertex->run();
  return 0;
}
