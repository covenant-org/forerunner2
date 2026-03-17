#include "argument_parser.hpp"
#include "crazybridge.hpp"
#include "crazyflie_cpp/Crazyflie.h"
#include "message.hpp"
#include <cstdio>
#include <functional>

Logger logger;

CrazyBridge::CrazyBridge(Core::ArgumentParser args)
    : Core::Vertex(args),
      crf("radio://0/100/2M/E7E7E7E7E7", logger,
          std::bind(&CrazyBridge::cb, this, std::placeholders::_1)) {
  this->_pose_subscriber = this->create_subscriber<PoseStamped>(
      "optitrack/marker",
      std::bind(&CrazyBridge::pose_callback, this, std::placeholders::_1));
}

void CrazyBridge::run() {
  printf("%s\n", this->crf.getFirmwareVersion().c_str());
  char k;
  scanf("%c", &k);
  this->crf.takeoff(1, 1);
  this->crf.land(0, 1);
}

void CrazyBridge::pose_callback(const Core::IncomingMessage<PoseStamped>& msg) {
  auto pose = msg.content.getPose();
  auto position = pose.getPosition();
  auto x = position.getX();
  auto y = position.getY();
  auto z = position.getZ();
  this->_logger.debug("Received position %f, %f, %f", x, y, z);
  this->crf.sendExternalPositionUpdate(x, y, z);
}

void CrazyBridge::cb(const char* msg) { printf("%s\n", msg); }

int main(int argc, char** argv) {
  Core::BaseArgumentParser args(argc, argv);
  CrazyBridge node(args);
  node.run();
  return 0;
}
