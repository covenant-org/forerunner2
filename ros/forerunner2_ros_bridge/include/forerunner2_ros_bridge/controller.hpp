#include "argument_parser.hpp"
#include "capnp_schemas/controller.capnp.h"
#include "capnp_schemas/mavlink.capnp.h"
#include "message.hpp"
#include "publisher.hpp"
#include "px4_msgs/msg/actuator_motors.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vertex.hpp"
#include <capnp_schemas/generics.capnp.h>
#include <memory>

class RosController : public Core::Vertex, public rclcpp::Node {
 private:
  std::shared_ptr<Core::Publisher<Odometry>> _odom_pub;
  std::shared_ptr<Core::Publisher<Telemetry>> _tel_pub;
  std::shared_ptr<Core::ActionServer<Command, GenericResponse>> _server;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _odom_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _tel_sub;
  rclcpp::TimerBase::SharedPtr _timer;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
      _offboard_control_mode_publisher;
  rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr
      _actuators_publisher;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
      _vehicle_command_publisher;
  std::atomic<unsigned int> _offboard_setpoint_counter;
  std::vector<float> _actuators;

 public:
  explicit RosController(Core::ArgumentParser args);
  void handle_commands(const Core::IncomingMessage<Command> &command,
                       GenericResponse::Builder &res);
  void publish_vehicle_command(uint16_t command, float param1, float param2 = 0,
                               float param3 = 0);
  void publish_offboard_control_mode();
  void publish_control();
  void arm();
  void disarm();
};
