#include "message.hpp"
#include "subscriber.hpp"
#include "vertex.hpp"
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <capnp_schemas/mavlink.capnp.h>
#include <capnp_schemas/registry.capnp.h>
#include <memory>

class TestingVertex : public Core::Vertex {
 public:
  TestingVertex(int, char **);
  void run();
  void telemetry_cb(const Core::IncomingMessage<Telemetry> &);

 private:
  std::shared_ptr<Core::Subscriber<Telemetry>> _telemetry_subscriber;
};

void TestingVertex::telemetry_cb(const Core::IncomingMessage<Telemetry> &msg) {
  printf("Received %d\n", msg.content.getBattery().getPercentage());
}

TestingVertex::TestingVertex(int argc, char **argv) : Vertex(argc, argv) {
  this->_telemetry_subscriber = this->create_subscriber<Telemetry>(
      "telemetry",
      std::bind(&TestingVertex::telemetry_cb, this, std::placeholders::_1));
}

void TestingVertex::run() {
  while (true) {
    sleep(1);
  }
}

int main(int argc, char **argv) {
  auto testing = TestingVertex(argc, argv);
  testing.run();
  return 0;
}
