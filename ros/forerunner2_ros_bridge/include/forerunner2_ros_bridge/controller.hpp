#include "argument_parser.hpp"
#include "capnp_schemas/mavlink.capnp.h"
#include "publisher.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vertex.hpp"
#include <memory>

class RosController : public Core::Vertex, public rclcpp::Node {
 private:
  std::shared_ptr<Core::Publisher<Odometry>> _odom_pub;
  std::shared_ptr<Core::Publisher<Telemetry>> _tel_pub;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _odom_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _tel_sub;

 public:
  explicit RosController(Core::ArgumentParser args);
};
