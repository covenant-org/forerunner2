#include "argument_parser.hpp"
#include "message.hpp"
#include "subscriber.hpp"
#include "vertex.hpp"
#include <capnp_schemas/sensors.capnp.h>
#include <memory>
#include <unistd.h>

class Remote : Core::Vertex {
 private:
  std::shared_ptr<Core::Subscriber<StereoMic>> _mic_sub;
  void mic_cb(const Core::IncomingMessage<StereoMic> &msg) {
    this->_logger.info("mic: %d", msg.content.getLeft());
  }

 public:
  Remote(Core::ArgumentParser args) : Core::Vertex(args) {
    this->_mic_sub = this->create_subscriber<StereoMic>(
        "mic", std::bind(&Remote::mic_cb, this, std::placeholders::_1));
  }

  void run() {
    while (true) {
      sleep(1);
    }
  }
};

int main(int argc, char **argv) {
  Core::BaseArgumentParser args(argc, argv);

  Remote remote(args);
  remote.run();
  return 0;
}
