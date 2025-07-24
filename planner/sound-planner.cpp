#include "sound-planner.hpp"

SoundPlanner::SoundPlanner(int argc, char **argv) : Core::Vertex(argc, argv) {
  this->_logger.set_level(Core::LogLevel::DEBUG);
  this->_odmetry_sub = this->create_subscriber<Odometry>(
      "odometry",
      std::bind(&SoundPlanner::odometry_cb, this, std::placeholders::_1));
  this->_command_client =
      this->create_action_client<Command, GenericResponse>("controller");
}

void SoundPlanner::odometry_cb(const Core::IncomingMessage<Odometry> &msg) {
  this->_logger.debug("odom");
}

void SoundPlanner::run() {
  while (true) sleep(1);
}

int main(int argc, char **argv) {
  SoundPlanner planner(argc, argv);
  planner.run();
}
