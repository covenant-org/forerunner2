#include "argument_parser.hpp"
#include "lowlevel.hpp"
#include "message.hpp"
#include "utils.hpp"
#include "vertex.hpp"

LowLevel::LowLevel(Core::ArgumentParser args) : Core::Vertex(args) {
  this->_odom_sub = this->create_subscriber<Odometry>(
      "odometry", std::bind(&LowLevel::_odom_cb, this, std::placeholders::_1));
}

void LowLevel::_odom_cb(const Core::IncomingMessage<Odometry> &msg) {
  auto odom_q = msg.content.getQ();
  auto odom_pos = msg.content.getPosition();
  q = Eigen::Quaterniond(odom_q.getW(), odom_q.getX(), -odom_q.getY(),
                         -odom_q.getZ());
  pos = Eigen::Vector3d(odom_pos.getX(), -odom_q.getY(), -odom_pos.getZ());
}

void LowLevel::run() {
  Core::RateKeeper rk(100);
  while (true) {
    rk.keep();
  }
}

int main(int argc, char **argv) {
  Core::BaseArgumentParser args(argc, argv);
  LowLevel controller(args);
  controller.run();
  return 0;
}
