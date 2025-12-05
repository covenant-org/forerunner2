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
#include <mutex>
#include <unistd.h>

LowLevel::LowLevel(Core::ArgumentParser args) : Core::Vertex(args) {
  this->_odom_sub = this->create_subscriber<Odometry>(
      "odometry", std::bind(&LowLevel::_odom_cb, this, std::placeholders::_1));
  this->_metrics_pub =
      this->create_publisher<ControlMetrics>("controller/metrics");
  this->_goal_pub = this->create_publisher<Position>("goal");
  this->_mavlink_client =
      this->create_action_client<Command, GenericResponse>("controller");
  _q = Eigen::Quaterniond(1, 0, 0, 0);
  _pos = Eigen::Vector3d(0, 0, 0);
  _linear_velocity = Eigen::Vector3d(0, 0, 0);
  _angular_velocity = Eigen::Vector3d(0, 0, 0);
  _g = Eigen::Vector3d(0, 0, -9.81);
  _m = 0.216;
  _f = Eigen::Vector3d(0, 0, 0);
  auto kpt = this->get_argument<double>("--kpt");
  auto kdt = this->get_argument<double>("--kdt");
  _kpt << kpt, 0, 0, 0, kpt, 0, 0, 0, 0.4;
  _kdt << kdt, 0, 0, 0, kdt, 0, 0, 0, 0.31;
  auto kpr = this->_args.get_argument<double>("--kpr");
  auto kdr = this->_args.get_argument<double>("--kdr");
  _kpr << -1 * kpr, 0, 0, 0, kpr, 0, 0, 0, kpr;
  _kdr << -1 * kdr, 0, 0, 0, kdr, 0, 0, 0, kdr;
  _J << 0.0216, 0, 0, 0, 0.0216, 0, 0, 0, 0.04;
}

void LowLevel::_odom_cb(const Core::IncomingMessage<Odometry>& msg) {
  const std::lock_guard<std::mutex> lock(_odom_mutex);
  auto odom_q = msg.content.getQ();
  auto odom_pos = msg.content.getPosition();
  auto velocity = msg.content.getVelocity();
  auto angular = msg.content.getAngular();
  _q = Eigen::Quaterniond(odom_q.getW(), odom_q.getX(), odom_q.getY(),
                          odom_q.getZ());
  _q.normalize();
  _pos = Eigen::Vector3d(odom_pos.getX(), odom_pos.getY(), odom_pos.getZ());
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
  Eigen::Vector3d r(q.x(), q.y(), q.z());
  auto norm = r.norm();
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

  _qd =
      Eigen::Quaterniond(real, -imaginary.x(), -imaginary.y(), -imaginary.z());
  _qe = qz.conjugate() * _qd.conjugate() * this->_q;
  _qe.normalize();
  auto theta = 2 * std::acos(_qe.w());
  Eigen::Vector3d qrv = (theta / std::sin(theta / 2)) *
                        Eigen::Vector3d(_qe.x(), _qe.y(), _qe.z());
  if (qrv.norm() > M_PI) {
    this->_logger.info("grater than pi");
    _qd = Eigen::Quaterniond(-_qd.w(), -_qd.x(), -_qd.y(), -_qd.z());
    _qe = qz.conjugate() * _qd.conjugate() * this->_q;
    _qe.normalize();
  }

  return -2 * _kpr * quaternion_ln(_qe) - _kdr * this->_angular_velocity +
         this->_angular_velocity.cross(this->_J * this->_angular_velocity);
}

float sqrt_n_trim(float input) {
  auto speed = std::sqrt(std::max(input, 0.0f));
  //  auto pwm = std::sqrt(speed / 1000);
  return std::min(speed, 1.0f);
}

void LowLevel::run() {
  Core::RateKeeper rk(100);
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
  float actuators[4];
  while (true) {
    Eigen::Vector3d pd{3, -4, 7};
    auto goal_msg = this->_goal_pub->new_msg();
    goal_msg.content.setX(pd.x());
    goal_msg.content.setY(pd.y());
    goal_msg.content.setZ(pd.z());
    goal_msg.publish();
    double fth = 0;
    Eigen::Vector3d ut, tau, fu, error;
    {
      const std::lock_guard<std::mutex> lock(_odom_mutex);
      ut = this->get_f_desired(pd);
      fth = ut.norm();
      fu = ut / fth;
      auto ur = this->get_torque_input(fu, 0);
      tau = this->_J * ur;
      error = (this->_pos - pd);
    }
    auto actuator_ctl = this->_mavlink_client->new_msg();
    auto ctl = actuator_ctl.content.initSetActuators(4);
    this->_logger.info("fth: %f tx: %f ty: %f tz: %f", fth, tau[0], tau[1],
                       tau[2]);
    float factor = 1.0 / 4.0;
    float real_pwm[4];
    tau[0] /= 1;
    tau[1] = -1 * tau[1] / 1;
    tau[2] = -1 * tau[2] / 1;
    real_pwm[0] = factor * (fth + tau[0] + tau[1] + tau[2]);
    real_pwm[1] = factor * (fth - tau[0] - tau[1] + tau[2]);
    real_pwm[3] = factor * (fth + tau[0] - tau[1] - tau[2]);
    real_pwm[2] = factor * (fth - tau[0] + tau[1] - tau[2]);
    actuators[0] = sqrt_n_trim(real_pwm[0]);
    actuators[1] = sqrt_n_trim(real_pwm[1]);
    actuators[2] = sqrt_n_trim(real_pwm[2]);
    actuators[3] = sqrt_n_trim(real_pwm[3]);
    ctl.set(0, actuators[0]);
    ctl.set(1, actuators[1]);
    ctl.set(2, actuators[2]);
    ctl.set(3, actuators[3]);
    actuator_ctl.send();
    this->_logger.info("Command sent");
    auto msg = this->_metrics_pub->new_msg();
    auto e = msg.content.initError(3);
    e.set(0, error[0]);
    e.set(1, error[1]);
    e.set(2, error[2]);
    auto qd_msg = msg.content.initQd(4);
    qd_msg.set(0, _qd.w());
    qd_msg.set(1, _qd.x());
    qd_msg.set(2, _qd.y());
    qd_msg.set(3, _qd.z());
    auto qe_msg = msg.content.initQe(4);
    qe_msg.set(0, _qe.w());
    qe_msg.set(1, _qe.x());
    qe_msg.set(2, _qe.y());
    qe_msg.set(3, _qe.z());
    auto pwm_msg = msg.content.initPwm(4);
    pwm_msg.set(0, actuators[0]);
    pwm_msg.set(1, actuators[1]);
    pwm_msg.set(2, actuators[2]);
    pwm_msg.set(3, actuators[3]);
    auto thrust_msg = msg.content.initThrust(3);
    thrust_msg.set(0, tau[0]);
    thrust_msg.set(1, tau[1]);
    thrust_msg.set(2, tau[2]);
    auto fu_msg = msg.content.initFu(3);
    fu_msg.set(0, fu[0]);
    fu_msg.set(1, fu[1]);
    fu_msg.set(2, fu[2]);
    msg.publish();
    rk.keep();
  }
}

int main(int argc, char** argv) {
  Core::BaseArgumentParser args(argc, argv);
  args.add_argument("--kpr").default_value(22.0).scan<'g', double>();
  args.add_argument("--kdr").default_value(15.0).scan<'g', double>();
  args.add_argument("--kpt").default_value(0.15).scan<'g', double>();
  args.add_argument("--kdt").default_value(0.05).scan<'g', double>();
  LowLevel controller(args);
  controller.run();
  return 0;
}
