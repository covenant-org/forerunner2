#include "argument_parser.hpp"
#include "message.hpp"
#include "subscriber.hpp"
#include "vertex.hpp"
#include <capnp_schemas/zed.capnp.h>
#include <memory>
#include <unistd.h>

class Remote : Core::Vertex {
 private:
  std::shared_ptr<Core::Subscriber<PointCloud>> _pt_sub;
  void pt_cb(const Core::IncomingMessage<PointCloud> &msg) {
    this->_logger.info("pt of size: %d", msg.content.getSize());
  }

 public:
  Remote(Core::ArgumentParser args) : Core::Vertex(args) {
    this->_pt_sub = this->create_subscriber<PointCloud>(
        "point_cloud", std::bind(&Remote::pt_cb, this, std::placeholders::_1));
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
