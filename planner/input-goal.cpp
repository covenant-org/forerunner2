#include "argument_parser.hpp"
#include "publisher.hpp"
#include "vertex.hpp"
#include <capnp_schemas/mavlink.capnp.h>
#include <chrono>
#include <thread>

class InputGoal : public Core::Vertex {
 private:
  std::shared_ptr<Core::Publisher<Position>> _position_pub;

 public:
  InputGoal(Core::ArgumentParser parser) : Core::Vertex(parser) {
    this->_position_pub = this->create_publisher<Position>("input_goal");
  }

  void run() override {
    while (true) {
      auto msg = this->_position_pub->new_msg();
      float x, y, z;
      std::cin >> x >> y >> z;
      msg.content.setX(x);
      msg.content.setY(y);
      msg.content.setZ(z);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      msg.publish();
    }
  }
};

int main(int argc, char** argv) {
  Core::BaseArgumentParser parser(argc, argv);

  std::shared_ptr<InputGoal> vertex = std::make_shared<InputGoal>(parser);
  vertex->run();
  return 0;
}
