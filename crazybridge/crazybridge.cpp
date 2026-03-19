#include "argument_parser.hpp"
#include "crazybridge.hpp"
#include "crazyflie_cpp/Crazyflie.h"
#include "message.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <cstdio>
#include <functional>
#include <memory>
#include <unistd.h>

CrazyBridge::CrazyBridge(Core::ArgumentParser args)
    : Core::Vertex(args),
      crf("radio://0/100/2M/E7E7E7E7E7", this->clogger,
          std::bind(&CrazyBridge::cb, this, std::placeholders::_1)) {
  printf("%s\n", this->crf.getFirmwareVersion().c_str());
  this->crf.requestParamToc();
  this->crf.requestLogToc();
  this->crf.setParam(51, 1);
  this->crf.setParam(157, 2);
  this->_pose_subscriber = this->create_subscriber<PoseStamped>(
      "optitrack/marker",
      std::bind(&CrazyBridge::pose_callback, this, std::placeholders::_1));
  this->_odom_publisher = this->create_publisher<Odometry>("odometry");

  std::function<void(uint32_t, std::vector<double>*, void*)> qLogCB =
      std::bind(&onLogQuartenion, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3);
  this->qLog = std::make_unique<LogBlockGeneric>(
      &this->crf,
      std::vector<std::string>{"kalman.stateX", "kalman.stateY",
                               "kalman.stateZ"},
      this, qLogCB);

  std::function<void(uint32_t, std::vector<double>*, void*)> posLogCB =
      std::bind(&onLogPosition, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3);
  this->qLog = std::make_unique<LogBlockGeneric>(
      &this->crf,
      std::vector<std::string>{"kalman.q0", "kalman.q1", "kalman.q2",
                               "kalman.q3"},
      this, posLogCB);
}

void onLogQuartenion(uint32_t time_in_ms, std::vector<double>* values,
                     void* data) {
  (void)time_in_ms;
  if (data == nullptr) return;
  CrazyBridge* bridge = (CrazyBridge*)data;
  if (values->size() != 4) return;
  bridge->_pose.q = Eigen::Quaterniond(values->at(0), values->at(1),
                                       values->at(2), values->at(3));
  bridge->publish_odom();
}

void onLogPosition(uint32_t time_in_ms, std::vector<double>* values,
                   void* data) {
  (void)time_in_ms;
  if (data == nullptr) return;
  CrazyBridge* bridge = (CrazyBridge*)data;
  if (values->size() != 3) return;
  bridge->_pose.pos =
      Eigen::Vector3d(values->at(0), values->at(1), values->at(2));
  bridge->publish_odom();
}

void CrazyBridge::publish_odom() {
  auto msg = this->_odom_publisher->new_msg();
  auto q = msg.content.initQ();
  auto pos = msg.content.initPosition();
  q.setX(this->_pose.q.x());
  q.setY(this->_pose.q.y());
  q.setZ(this->_pose.q.z());
  q.setW(this->_pose.q.w());
  pos.setX(this->_pose.pos.x());
  pos.setY(this->_pose.pos.y());
  pos.setZ(this->_pose.pos.z());
  msg.publish();
}

void CrazyBridge::pose_callback(const Core::IncomingMessage<PoseStamped>& msg) {
  auto pose = msg.content.getPose();
  auto position = pose.getPosition();
  auto orientation = pose.getOrientation();
  auto x = (float)position.getX();
  auto y = (float)position.getY();
  auto z = (float)position.getZ();
  auto qx = (float)orientation.getX();
  auto qy = (float)orientation.getY();
  auto qz = (float)orientation.getZ();
  auto qw = (float)orientation.getW();
  this->_logger.debug("Received position %f, %f, %f, %f, %f, %f\n", x, y, z, qx,
                      qz, qy, qw);
  // this->crf.sendExternalPoseUpdate(x, y, z, qx, qy, qz, qw);
  this->crf.sendExternalPositionUpdate(x, y, z);
}

void CrazyBridge::cb(const char* msg) { printf("%s\n", msg); }

void CrazyBridge::run() {
  this->qLog->start(10);
  this->posLog->start(10);
  char k;
  scanf("%c", &k);
  while (getchar() != '\n');
  this->crf.takeoff(2, 2);
  int i = 0;
  while (i < 100) {
    this->crf.goTo(0, 0, 0, 0, 2);
    usleep(50000);
  }
  this->crf.land(0, 1);
  this->qLog->stop();
  this->posLog->stop();
}

int main(int argc, char** argv) {
  Core::BaseArgumentParser args(argc, argv);
  CrazyBridge node(args);
  node.run();
  return 0;
}
