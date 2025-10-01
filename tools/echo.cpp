// Generado por Claudio

#include "argument_parser.hpp"
#include "vertex.hpp"
#include "subscriber.hpp"
#include <capnp_schemas/std_msgs.capnp.h>
#include <capnp_schemas/generics.capnp.h>
#include <capnp_schemas/mavlink.capnp.h>
#include <capnp_schemas/sensors.capnp.h>
#include <capnp_schemas/geometry_msgs.capnp.h>
#include <capnp_schemas/zed.capnp.h>
#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <chrono>

class Echo : public Core::Vertex {
private:
    std::string _topic;
    std::string _message_type;
    
public:
    Echo(Core::ArgumentParser parser) : Core::Vertex(parser) {
        _topic = this->get_argument<std::string>("topic");
        _message_type = this->get_argument<std::string>("--type");
        
        this->_logger.info("Echo listening to topic: %s (type: %s)", _topic.c_str(), _message_type.c_str());
        
        // Subscribe based on message type
        if (_message_type == "KeyValue") {
            auto subscriber = this->create_subscriber<KeyValue>(_topic,
                [this](const Core::IncomingMessage<KeyValue>& msg) {
                    std::cout << "[" << _topic << "] " << msg.content.getKey().cStr() 
                              << " = " << msg.content.getValue().cStr() << std::endl;
                });
        }
        else if (_message_type == "HomePosition") {
            auto subscriber = this->create_subscriber<HomePosition>(_topic,
                [this](const Core::IncomingMessage<HomePosition>& msg) {
                    auto pos = msg.content.getPos();
                    std::cout << "[" << _topic << "] POS: x=" << pos.getX() 
                              << " y=" << pos.getY() 
                              << " z=" << pos.getZ() << std::endl;
                });
        }
        else if (_message_type == "Telemetry") {
            auto subscriber = this->create_subscriber<Telemetry>(_topic,
                [this](const Core::IncomingMessage<Telemetry>& msg) {
                    auto battery = msg.content.getBattery();
                    std::cout << "[" << _topic << "] Armed: " << (msg.content.getArmed() ? "true" : "false")
                              << " | Battery: " << battery.getPercentage() << "%"
                              << " | In Air: " << (msg.content.getInAir() ? "true" : "false")
                              << " | Mode: " << msg.content.getMode().cStr() << std::endl;
                });
        }
        else if (_message_type == "Odometry") {
            auto subscriber = this->create_subscriber<Odometry>(_topic,
                [this](const Core::IncomingMessage<Odometry>& msg) {
                    auto pos = msg.content.getPosition();
                    auto vel = msg.content.getVelocity();
                    auto q = msg.content.getQ();
                    std::cout << "[" << _topic << "] POS: (" << pos.getX() << ", " << pos.getY() << ", " << pos.getZ() << ")"
                              << " | VEL: (" << vel.getX() << ", " << vel.getY() << ", " << vel.getZ() << ")"
                              << " | Q: (" << q.getW() << ", " << q.getX() << ", " << q.getY() << ", " << q.getZ() << ")"
                              << " | Heading: " << msg.content.getHeading() << "°" << std::endl;
                });
        }
        else if (_message_type == "Altitude") {
            auto subscriber = this->create_subscriber<Altitude>(_topic,
                [this](const Core::IncomingMessage<Altitude>& msg) {
                    std::cout << "[" << _topic << "] Local: " << msg.content.getLocal() << "m"
                              << " | Relative: " << msg.content.getRelative() << "m"
                              << " | Monotonic: " << msg.content.getMonotonic() << "m"
                              << " | Avg: " << msg.content.getAvg() << "m" << std::endl;
                });
        }
        else if (_message_type == "Point") {
            auto subscriber = this->create_subscriber<Point>(_topic,
                [this](const Core::IncomingMessage<Point>& msg) {
                    std::cout << "[" << _topic << "] (" << msg.content.getX() 
                              << ", " << msg.content.getY() 
                              << ", " << msg.content.getZ() << ")" << std::endl;
                });
        }
        else if (_message_type == "PointCloud") {
            auto subscriber = this->create_subscriber<PointCloud>(_topic,
                [this](const Core::IncomingMessage<PointCloud>& msg) {
                    std::cout << "[" << _topic << "] Width: " << msg.content.getWidth()
                              << " | Height: " << msg.content.getHeight()
                              << " | Size: " << msg.content.getSize() << " bytes"
                              << " | Data length: " << msg.content.getData().size() << std::endl;
                });
        }
        else {
            this->_logger.error("Unknown message type: %s", _message_type.c_str());
            this->_logger.info("Supported types: KeyValue, HomePosition, Telemetry, Odometry, Altitude, Point, PointCloud");
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
    
    parser.add_argument("topic")
        .help("Topic name to subscribe to")
        .required();
    
    parser.add_argument("--type", "-t")
        .help("Message type (KeyValue, HomePosition, Telemetry, Odometry, Altitude, Point, PointCloud)")
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
