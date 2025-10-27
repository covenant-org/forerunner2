// Generado por Claudio

#include "argument_parser.hpp"
#include "vertex.hpp"
#include <capnp_schemas/generics.capnp.h>
#include <capnp_schemas/geometry_msgs.capnp.h>
#include <capnp_schemas/mavlink.capnp.h>
#include <capnp_schemas/sensors.capnp.h>
#include <capnp_schemas/std_msgs.capnp.h>
#include <capnp_schemas/zed.capnp.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

class Echo : public Core::Vertex {
 private:
  std::string _topic;
  std::string _message_type;

 public:
  Echo(Core::ArgumentParser parser) : Core::Vertex(parser) {
    _topic = this->get_argument<std::string>("topic");
    _message_type = this->get_argument<std::string>("--type");

    this->_logger.info("Echo listening to topic: %s (type: %s)", _topic.c_str(),
                       _message_type.c_str());

    // Subscribe based on message type
    if (_message_type == "KeyValue") {
      auto subscriber = this->create_subscriber<KeyValue>(
          _topic, [this](const Core::IncomingMessage<KeyValue>& msg) {
            this->_logger.info("[%s] %s = %s", _topic.c_str(),
                               msg.content.getKey().cStr(),
                               msg.content.getValue().cStr());
          });
    } else if (_message_type == "HomePosition") {
      auto subscriber = this->create_subscriber<HomePosition>(
          _topic, [this](const Core::IncomingMessage<HomePosition>& msg) {
            auto pos = msg.content.getPos();
            this->_logger.info("[%s] POS: x=%f y=%f z=%f", _topic.c_str(),
                               pos.getX(), pos.getY(), pos.getZ());
          });
    } else if (_message_type == "Telemetry") {
      auto subscriber = this->create_subscriber<Telemetry>(
          _topic, [this](const Core::IncomingMessage<Telemetry>& msg) {
            auto battery = msg.content.getBattery();
            this->_logger.info(
                "[%s] Armed: %s | Battery: %d | In Air: %s | Mode: %s",
                _topic.c_str(), (msg.content.getArmed() ? "true" : "false"),
                battery.getPercentage(),
                (msg.content.getInAir() ? "true" : "false"),
                msg.content.getMode().cStr());
          });
    } else if (_message_type == "Odometry") {
      auto subscriber = this->create_subscriber<Odometry>(
          _topic, [this](const Core::IncomingMessage<Odometry>& msg) {
            auto pos = msg.content.getPosition();
            auto vel = msg.content.getVelocity();
            auto q = msg.content.getQ();
            this->_logger.info(
                "[%s] POS: (%f, %f, %f) | VEL: (%f, %f, %f) | Q: (%f, %f, %f, "
                "%f) | Heading: %f",
                _topic.c_str(), pos.getX(), pos.getY(), pos.getZ(), vel.getX(),
                vel.getY(), vel.getZ(), q.getW(), q.getX(), q.getY(), q.getZ(),
                msg.content.getHeading());
          });
    } else if (_message_type == "Altitude") {
      auto subscriber = this->create_subscriber<Altitude>(
          _topic, [this](const Core::IncomingMessage<Altitude>& msg) {
            this->_logger.info(
                "[%s] Local: %fm | Relative: %fm | Monotonic: %fm | Avg: %fm",
                _topic.c_str(), msg.content.getLocal(),
                msg.content.getRelative(), msg.content.getMonotonic(),
                msg.content.getAvg());
          });
    } else if (_message_type == "Point") {
      auto subscriber = this->create_subscriber<Point>(
          _topic, [this](const Core::IncomingMessage<Point>& msg) {
            this->_logger.info("[%s] (%f, %f, %f)", _topic.c_str(),
                               msg.content.getX(), msg.content.getY(),
                               msg.content.getZ());
          });
    } else if (_message_type == "PointCloud") {
      auto subscriber = this->create_subscriber<PointCloud>(
          _topic, [this](const Core::IncomingMessage<PointCloud>& msg) {
            this->_logger.info(
                "[%s] Width: %d | Height: %d | Size: %d | Data length: %d",
                _topic.c_str(), msg.content.getWidth(), msg.content.getHeight(),
                msg.content.getSize(), msg.content.getData().size());
          });
    } else {
      this->_logger.error("Unknown message type: %s", _message_type.c_str());
      this->_logger.info(
          "Supported types: KeyValue, HomePosition, Telemetry, Odometry, "
          "Altitude, Point, PointCloud");
      throw std::runtime_error("Unsupported message type");
    }
  }

  void run() override {
    this->_logger.info("Echo started. Press Ctrl+C to stop.");
    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
};

int main(int argc, char** argv) {
  Core::BaseArgumentParser parser(argc, argv);

  parser.add_argument("topic").help("Topic name to subscribe to").required();

  parser.add_argument("--type", "-t")
      .help(
          "Message type (KeyValue, HomePosition, Telemetry, Odometry, "
          "Altitude, Point, PointCloud)")
      .default_value("KeyValue");

  try {
    std::shared_ptr<Echo> echo = std::make_shared<Echo>(parser);
    echo->run();
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
