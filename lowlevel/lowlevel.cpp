#include "argument_parser.hpp"
#include "lowlevel.hpp"
#include "message.hpp"
#include "utils.hpp"
#include "vertex.hpp"
#include <Eigen/src/Core/GlobalFunctions.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Scaling.h>
#include <capnp_schemas/controller.capnp.h>
#include <capnp_schemas/generics.capnp.h>
#include <cmath>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <unistd.h>

LowLevel::LowLevel(Core::ArgumentParser args) : Core::Vertex(args) {
  this->_odom_sub = this->create_subscriber<Odometry>(
      "odometry", std::bind(&LowLevel::_odom_cb, this, std::placeholders::_1));
  this->_mavlink_client =
      this->create_action_client<Command, GenericResponse>("controller");
  _q = Eigen::Quaterniond(1, 0, 0, 0);
  _pos = Eigen::Vector3d(0, 0, 0);
  _linear_velocity = Eigen::Vector3d(0, 0, 0);
  _angular_velocity = Eigen::Vector3d(0, 0, 0);
  _g = Eigen::Vector3d(0, 0, -9.81);
  _m = 0.3;
  _f = Eigen::Vector3d(0, 0, 0);
  _kpt << 0.05, 0, 0, 0, 0.05, 0, 0, 0, 0.001;
  _kdt << 0.0, 0, 0, 0, 0.0, 0, 0, 0, 0.04;
  _J << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
  _kpr << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
  _kdr << 0, 0, 0, 0, 0, 0, 0, 0, 0;
}

void LowLevel::_odom_cb(const Core::IncomingMessage<Odometry> &msg) {
  auto odom_q = msg.content.getQ();
  auto odom_pos = msg.content.getPosition();
  auto velocity = msg.content.getVelocity();
  auto angular = msg.content.getAngular();
  _q = Eigen::Quaterniond(odom_q.getW(), odom_q.getX(), odom_q.getY(),
                          odom_q.getZ());
  _q.normalize();
  _pos = Eigen::Vector3d(odom_pos.getX(), odom_q.getY(), odom_pos.getZ());
  _linear_velocity =
      Eigen::Vector3d(velocity.getX(), velocity.getY(), velocity.getZ());
  _angular_velocity =
      Eigen::Vector3d(angular.getX(), angular.getY(), angular.getZ());
}

Eigen::Quaterniond LowLevel::get_q_desired(Eigen::Vector3d f, double yaw) {
  static Eigen::Vector3d nz(0, 0, 1);
  auto dot = f.dot(nz);
  auto real = std::sqrt((1 + dot) / 2);
  Eigen::Vector3d cross = f.cross(nz);
  Eigen::Vector3d imaginary = (cross / cross.norm()) * std::sqrt((1 - dot) / 2);
  return Eigen::Quaterniond(real, imaginary.x(), imaginary.y(), imaginary.z()) *
         Eigen::Quaterniond(std::cos(yaw / 2), 0, 0, std::sin(yaw / 2));
}

Eigen::Quaterniond LowLevel::get_q_error(Eigen::Quaterniond qd) {
  return qd.conjugate() * this->_q;
}

Eigen::Vector3d LowLevel::get_f_desired(Eigen::Vector3d pd) {
  return -_kpt * (_pos - pd) - _kdt * _linear_velocity - _m * _g;
}

Eigen::Vector3d quaternion_ln(const Eigen::Quaterniond q) {
  Eigen::Quaterniond quart(q);
  quart.normalize();
  auto norm = quart.norm();
  if (norm == 0) {
    return Eigen::Vector3d(0, 0, 0);
  }
  return (Eigen::Vector3d(quart.x(), quart.y(), quart.z()) / norm) *
         std::acos(quart.w());
}

Eigen::Vector3d LowLevel::get_torque_input(Eigen::Vector3d fu, double yaw) {
  Eigen::Quaterniond qz(std::cos(yaw / 2), 0, 0, std::sin(yaw / 2));
  Eigen::Vector3d unitz(0, 0, 1);
  auto dot = fu.dot(unitz);
  Eigen::Vector3d cross = fu.cross(unitz);
  Eigen::Vector3d imaginary(0, 0, 0);
  if (cross.norm() != 0) {
    imaginary = (cross / cross.norm()) * std::sqrt((1 - dot) / 2);
  }
  auto real = std::sqrt((1 + dot) / 2);
  auto qd =
      Eigen::Quaterniond(real, imaginary.x(), imaginary.y(), imaginary.z());
  auto qe = qz * qd.conjugate() * this->_q;
  qe.normalize();
  auto theta = 2 * std::acos(qe.w());
  Eigen::Vector3d qrv =
      (theta / std::sin(theta / 2)) * Eigen::Vector3d(qe.x(), qe.y(), qe.z());
  if (qrv.norm() > M_PI) {
    qd = qd * Eigen::UniformScaling<double>(-1);
    qe = qz * qd.conjugate() * this->_q;
    qe.normalize();
  }

  return -2 * _kpr * quaternion_ln(qe) - _kdr * this->_angular_velocity +
         this->_angular_velocity.cross(this->_J * this->_angular_velocity);
}

void LowLevel::run() {
  Core::RateKeeper rk(1000);
  auto command = this->_mavlink_client->new_msg();
  command.content.setArm();
  auto res = command.send();
  if (!res.has_value()) {
    this->_logger.error("No response recieved from mavlink");
    return;
  }
  if (res.value().content.getCode() != 200) {
    this->_logger.error("Error arming: %s",
                        res.value().content.getMessage().cStr());
    return;
  }
  this->_logger.info("Armed");
  sleep(1);
  auto first_actuator_ctl = this->_mavlink_client->new_msg();
  auto ctl = first_actuator_ctl.content.initSetActuators(4);
  ctl.set(0, 0);
  ctl.set(1, 0);
  ctl.set(2, 0);
  ctl.set(3, 0);
  auto res_ctl = first_actuator_ctl.send();
  if (res_ctl.value().content.getCode() != 200) {
    this->_logger.error("Error sending first actuator control");
    return;
  }
  auto enable_offboard = this->_mavlink_client->new_msg();
  auto offboard = enable_offboard.content.initOffboard();
  offboard.setEnable(true);
  auto res_offboard = enable_offboard.send();
  if (res_offboard.value().content.getCode() != 200) {
    this->_logger.error("Error setting to offboard: %s",
                        res_offboard.value().content.getMessage().cStr());
    return;
  }
  while (true) {
    auto ut = this->get_f_desired({0, 0, 7});
    auto fth = ut.norm();
    Eigen::Vector3d fu = ut / fth;
    auto ur = this->get_torque_input(fu, 0);
    Eigen::Vector3d tau = this->_J * ur;
    auto actuator_ctl = this->_mavlink_client->new_msg();
    auto ctl = actuator_ctl.content.initSetActuators(4);
    this->_logger.info("fth: %f tx: %f ty: %f tz: %f", fth, tau[0], tau[1],
                       tau[2]);
    this->_logger.info("Error :%f",
                       (this->_pos - Eigen::Vector3d(0, 0, 7)).norm());
    ctl.set(0,
            std::min(std::sqrt(std::max(
                         (1.0 / 4.0) * (fth + tau[0] + tau[1] + tau[2]), 0.0)),
                     1.0));
    ctl.set(1,
            std::min(std::sqrt(std::max(
                         (1.0 / 4.0) * (fth - tau[0] - tau[1] + tau[2]), 0.0)),
                     1.0));
    ctl.set(2,
            std::min(std::sqrt(std::max(
                         (1.0 / 4.0) * (fth - tau[0] + tau[1] - tau[2]), 0.0)),
                     1.0));
    ctl.set(3,
            std::min(std::sqrt(std::max(
                         (1.0 / 4.0) * (fth + tau[0] - tau[1] - tau[2]), 0.0)),
                     1.0));
    actuator_ctl.send();
    this->_logger.info("Command sent");
    rk.keep();
  }
}

int main(int argc, char **argv) {
  Core::BaseArgumentParser args(argc, argv);
  LowLevel controller(args);
  controller.run();
  return 0;
}
