#include "publisher.hpp"
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

 private:
  std::shared_ptr<Core::Publisher<Telemetry>> _telemetry_publisher;
  std::shared_ptr<Core::Publisher<Odometry>> _odometry_publisher;
};

TestingVertex::TestingVertex(int argc, char **argv) : Vertex(argc, argv) {
  this->_telemetry_publisher =
      this->create_publisher<Telemetry>("telemetry", 5551);
  this->_odometry_publisher =
      this->create_publisher<Odometry>("odometry", 5552);
}

void TestingVertex::run() {
  while (true) {
    ::capnp::MallocMessageBuilder builder;
    auto msg = builder.initRoot<Telemetry>();
    msg.initBattery().setPercentage(100);
    this->_telemetry_publisher->publish(builder);

    sleep(1);
  }
}

int main(int argc, char **argv) {
  auto testing = TestingVertex(argc, argv);
  testing.run();
  return 0;
}
