#include "argument_parser.hpp"
#include "crazybridge.hpp"
#include "crazyflie_cpp/Crazyflie.h"
#include "message.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <cstdio>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

CrazyBridge::CrazyBridge(Core::ArgumentParser args)
    : Core::Vertex(args),
      crf("radio://0/80/2M/E7E7E7E7E7", this->clogger,
          std::bind(&CrazyBridge::cb, this, std::placeholders::_1)) {
  this->_logger.info("Firmware: %s\n", this->crf.getFirmwareVersion().c_str());
  this->crf.requestParamToc();
  this->crf.requestLogToc();

#ifdef OOT
  this->_logger.info("Reading file");
  std::fstream pid_params_file("./pid.conf");
  this->_logger.info("Setting PID translation parameters");
  std::string kp_x_str, kp_y_str, kp_z_str, kd_x_str, kd_y_str, kd_z_str,
      emptyline;

  std::getline(pid_params_file, emptyline);
  this->_logger.debug("%s", emptyline.c_str());

  std::getline(pid_params_file, kp_x_str);
  std::getline(pid_params_file, kp_y_str);
  std::getline(pid_params_file, kp_z_str);

  std::getline(pid_params_file, emptyline);
  this->_logger.debug("%s", emptyline.c_str());

  std::getline(pid_params_file, kd_x_str);
  std::getline(pid_params_file, kd_y_str);
  std::getline(pid_params_file, kd_z_str);
  this->_logger.debug("Done reading file");
  auto trans_kp_x = this->crf.getParamTocEntry("ootParams", "trans_kp_x");
  auto trans_kp_y = this->crf.getParamTocEntry("ootParams", "trans_kp_y");
  auto trans_kp_z = this->crf.getParamTocEntry("ootParams", "trans_kp_z");
  this->_logger.debug("Done reading kp param entries");
  auto trans_kd_x = this->crf.getParamTocEntry("ootParams", "trans_kd_x");
  auto trans_kd_y = this->crf.getParamTocEntry("ootParams", "trans_kd_y");
  auto trans_kd_z = this->crf.getParamTocEntry("ootParams", "trans_kd_z");
  this->_logger.debug("Done reading param entries");
  this->crf.setParam(trans_kp_x->id, std::stof(kp_x_str));
  this->crf.setParam(trans_kp_y->id, std::stof(kp_y_str));
  this->crf.setParam(trans_kp_z->id, std::stof(kp_z_str));
  this->crf.setParam(trans_kd_x->id, std::stof(kd_x_str));
  this->crf.setParam(trans_kd_y->id, std::stof(kd_y_str));
  this->crf.setParam(trans_kd_z->id, std::stof(kd_z_str));

  this->_logger.info("Setting PID rotation parameters");

  std::getline(pid_params_file, emptyline);

  std::getline(pid_params_file, kp_x_str);
  std::getline(pid_params_file, kp_y_str);
  std::getline(pid_params_file, kp_z_str);

  std::getline(pid_params_file, emptyline);

  std::getline(pid_params_file, kd_x_str);
  std::getline(pid_params_file, kd_y_str);
  std::getline(pid_params_file, kd_z_str);
  auto rot_kp_x = this->crf.getParamTocEntry("ootParams", "rot_kp_x");
  auto rot_kp_y = this->crf.getParamTocEntry("ootParams", "rot_kp_y");
  auto rot_kp_z = this->crf.getParamTocEntry("ootParams", "rot_kp_z");
  auto rot_kd_x = this->crf.getParamTocEntry("ootParams", "rot_kd_x");
  auto rot_kd_y = this->crf.getParamTocEntry("ootParams", "rot_kd_y");
  auto rot_kd_z = this->crf.getParamTocEntry("ootParams", "rot_kd_z");
  this->crf.setParam(rot_kp_x->id, std::stof(kp_x_str));
  this->crf.setParam(rot_kp_y->id, std::stof(kp_y_str));
  this->crf.setParam(rot_kp_z->id, std::stof(kp_z_str));
  this->crf.setParam(rot_kd_x->id, std::stof(kd_x_str));
  this->crf.setParam(rot_kd_y->id, std::stof(kd_y_str));
  this->crf.setParam(rot_kd_z->id, std::stof(kd_z_str));
  this->_logger.info("Done configuring pid");
#endif  // OOT

  //  high level commander
  this->crf.setParam(96, 1);
  // kalman estimator
  this->crf.setParam(157, 2);
  this->_marker_subscriber = this->create_subscriber<TrackingMarker>(
      "optitrack/marker",
      std::bind(&CrazyBridge::marker_cb, this, std::placeholders::_1));
  this->_odom_publisher = this->create_publisher<Odometry>("odometry");

  std::function<void(uint32_t, std::vector<double>*, void*)> posErrorLogCB =
      std::bind(&onLogPosError, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3);
  std::function<void(uint32_t, std::vector<double>*, void*)> velErrorLogCB =
      std::bind(&onLogVelError, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3);
  std::function<void(uint32_t, std::vector<double>*, void*)> qErrorLogCB =
      std::bind(&onLogQError, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3);
  std::function<void(uint32_t, std::vector<double>*, void*)> angVelErrorLogCB =
      std::bind(&onLogAngVelError, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3);

  std::function<void(uint32_t, std::vector<double>*, void*)> qLogCB =
      std::bind(&onLogQuartenion, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3);
  std::function<void(uint32_t, std::vector<double>*, void*)> posLogCB =
      std::bind(&onLogPosition, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3);
  std::function<void(uint32_t, std::vector<double>*, void*)> qdLogCB =
      std::bind(&onLogQD, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3);
  this->posLog = std::make_unique<LogBlockGeneric>(
      &this->crf,
      std::vector<std::string>{"kalman.stateX", "kalman.stateY",
                               "kalman.stateZ"},
      this, posLogCB);

  this->qLog = std::make_unique<LogBlockGeneric>(
      &this->crf,
      std::vector<std::string>{"kalman.q0", "kalman.q1", "kalman.q2",
                               "kalman.q3"},
      this, qLogCB);
  this->_metrics_publisher =
      this->create_publisher<ControlMetrics>("controller/metrics");

#ifdef OOT
  this->posErrorLog = std::make_unique<LogBlockGeneric>(
      &this->crf,
      std::vector<std::string>{"oot.fth_x", "oot.fth_y", "oot.fth_z"}, this,
      posErrorLogCB);
  this->velErrorLog = std::make_unique<LogBlockGeneric>(
      &this->crf,
      std::vector<std::string>{"oot.vel_err_x", "oot.vel_err_y",
                               "oot.vel_err_z"},
      this, velErrorLogCB);
  this->qErrorLog = std::make_unique<LogBlockGeneric>(
      &this->crf,
      std::vector<std::string>{"oot.q_err_x", "oot.q_err_y", "oot.q_err_z",
                               "oot.q_err_w"},
      this, qErrorLogCB);
  this->qdLog = std::make_unique<LogBlockGeneric>(
      &this->crf,
      std::vector<std::string>{"oot.qd_x", "oot.qd_y", "oot.qd_z", "oot.qd_w"},
      this, qdLogCB);
  this->angVelErrorLog = std::make_unique<LogBlockGeneric>(
      &this->crf,
      std::vector<std::string>{"oot.ang_vel_err_x", "oot.ang_vel_err_y",
                               "oot.ang_vel_err_z"},
      this, angVelErrorLogCB);
#endif  // OOT
  this->_logger.debug("CrazyBridge setup ready");
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

void onLogQD(uint32_t time_in_ms, std::vector<double>* values, void* data) {
  (void)time_in_ms;
  if (data == nullptr) return;
  CrazyBridge* bridge = (CrazyBridge*)data;
  if (values->size() != 4) return;
  bridge->last_qd = Eigen::Quaterniond(values->at(3), values->at(0),
                                       values->at(1), values->at(2));
}

void onLogPosError(uint32_t time_in_ms, std::vector<double>* values,
                   void* data) {
  (void)time_in_ms;
  if (data == nullptr) return;
  if (values->size() != 3) return;
  CrazyBridge* bridge = (CrazyBridge*)data;
  bridge->last_pos_error =
      Eigen::Vector3d(values->at(0), values->at(1), values->at(2));
}

void onLogVelError(uint32_t time_in_ms, std::vector<double>* values,
                   void* data) {
  (void)time_in_ms;
  if (data == nullptr) return;
  if (values->size() != 3) return;
  // CrazyBridge* bridge = (CrazyBridge*)data;
}

void onLogQError(uint32_t time_in_ms, std::vector<double>* values, void* data) {
  (void)time_in_ms;
  if (data == nullptr) return;
  if (values->size() != 4) return;
  CrazyBridge* bridge = (CrazyBridge*)data;
  bridge->last_q_error = Eigen::Quaterniond(values->at(3), values->at(0),
                                            values->at(1), values->at(2));
}
void onLogAngVelError(uint32_t time_in_ms, std::vector<double>* values,
                      void* data) {
  (void)time_in_ms;
  if (data == nullptr) return;
  if (values->size() != 3) return;
  CrazyBridge* bridge = (CrazyBridge*)data;
  auto msg = bridge->_metrics_publisher->new_msg();
  msg.content.initFu(0);
  msg.content.initPwm(0);
  auto qd = msg.content.initQd(4);
  qd.set(1, bridge->last_qd.x());
  qd.set(2, bridge->last_qd.y());
  qd.set(3, bridge->last_qd.z());
  qd.set(0, bridge->last_qd.w());

  auto e = msg.content.initQe(4);
  e.set(1, bridge->last_q_error.x());
  e.set(2, bridge->last_q_error.y());
  e.set(3, bridge->last_q_error.z());
  e.set(0, bridge->last_q_error.w());

  auto error = msg.content.initError(3);
  error.set(0, (float)bridge->last_pos_error.x());
  error.set(1, (float)bridge->last_pos_error.y());
  error.set(2, (float)bridge->last_pos_error.z());

  msg.publish();
}

void onLogPosition(uint32_t time_in_ms, std::vector<double>* values,
                   void* data) {
  (void)time_in_ms;
  if (data == nullptr) return;
  if (values->size() != 3) return;
  CrazyBridge* bridge = (CrazyBridge*)data;
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

void CrazyBridge::marker_cb(const Core::IncomingMessage<TrackingMarker>& msg) {
  this->_logger.debug("pose callback");
  auto point = msg.content.getPoint();
  auto x = point.getX();
  auto y = point.getY();
  auto z = point.getZ();
  //  this->_logger.debug("Received position %f, %f, %f, %f, %f, %f\n", x, y, z,
  //  qx,
  //                      qz, qy, qw);
  // this->crf.sendExternalPoseUpdate(x, y, z, qx, qy, qz, qw);
  this->crf.sendExternalPositionUpdate(x, y, z);
}

void CrazyBridge::cb(const char* msg) { printf("%s\n", msg); }

void CrazyBridge::run() {
  this->qLog->start(5);
  this->posLog->start(5);

#ifdef OOT
  this->qErrorLog->start(1);
  this->qdLog->start(1);
  this->angVelErrorLog->start(1);
  this->posErrorLog->start(5);
  this->_logger.info("logging oot");
#endif  // OOT
  char k;
  scanf("%c", &k);
  while (getchar() != '\n');
  this->crf.takeoff(2, 2);
  sleep(3);
  //  this->crf.goTo(0.3, 0, 2, 0, 4);
  //  sleep(4);
  this->crf.land(0, 3);
  sleep(5);
  this->qLog->stop();
  this->posLog->stop();
}

int main(int argc, char** argv) {
  Core::BaseArgumentParser args(argc, argv);
  CrazyBridge node(args);
  node.run();
  return 0;
}
