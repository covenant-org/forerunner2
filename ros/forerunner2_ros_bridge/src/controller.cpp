#include "argument_parser.hpp"
#include "forerunner2_ros_bridge/controller.hpp"
#include <cstdio>
#include <iostream>
#include <memory>

RosController::RosController(Core::ArgumentParser args)
    : Core::Vertex(args), rclcpp::Node("RosController") {
  this->_logger.info("Constructor");
  this->_odom_pub = this->Core::Vertex::create_publisher<Odometry>("odometry");
  this->_tel_pub = this->Core::Vertex::create_publisher<Telemetry>("telemetry");
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5),
                         qos_profile);
  this->_odom_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", qos,
      [this](px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
        auto output = this->_odom_pub->new_msg();
        auto angular = output.content.initAngular();
        auto pos = output.content.initPosition();
        auto vel = output.content.initVelocity();
        auto q = output.content.initQ();
        angular.setX(msg->angular_velocity[0]);
        angular.setY(-msg->angular_velocity[1]);
        angular.setZ(-msg->angular_velocity[2]);
        pos.setX(msg->position[0]);
        pos.setY(-msg->position[1]);
        pos.setZ(-msg->position[2]);
        vel.setX(msg->velocity[0]);
        vel.setY(-msg->velocity[1]);
        vel.setZ(-msg->velocity[2]);
        q.setW(msg->q[0]);
        q.setX(msg->q[1]);
        q.setY(-msg->q[2]);
        q.setZ(-msg->q[3]);
        output.publish();
      });
  this->_tel_sub = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", qos,
      [this](px4_msgs::msg::VehicleStatus::UniquePtr msg) {
        auto output = this->_tel_pub->new_msg();
        output.content.setArmed(msg->arming_state == 2);
        output.publish();
      });
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  Core::BaseArgumentParser args(argc, argv);
  rclcpp::spin(std::make_shared<RosController>(args));
  rclcpp::shutdown();
  return 0;
}
