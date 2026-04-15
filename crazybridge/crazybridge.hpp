#include "argument_parser.hpp"
#include "capnp_schemas/controller.capnp.h"
#include "capnp_schemas/mavlink.capnp.h"
#include "capnp_schemas/optitrack.capnp.h"
#include "crazyflie_cpp/Crazyflie.h"
#include "message.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"
#include "vertex.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Geometry/Quaternion.h>
#include <memory>

#ifndef CRAZYBRIDGE_HPP
#define CRAZYBRIDGE_HPP

struct InternalPose {
  Eigen::Quaterniond q;
  Eigen::Vector3d pos;
};

class CustomLogger : public Logger {
 public:
  CustomLogger() {}

  void info(const std::string& msg) { printf("%s\n", msg.c_str()); }
  void warning(const std::string& msg) { printf("%s\n", msg.c_str()); }
  void error(const std::string& msg) { printf("%s\n", msg.c_str()); }
};

class CrazyBridge : public Core::Vertex {
 private:
  CustomLogger clogger;
  Crazyflie crf;
  std::shared_ptr<Core::Subscriber<TrackingMarker>> _marker_subscriber;
  std::shared_ptr<Core::Publisher<Odometry>> _odom_publisher;
  void cb(const char*);
  std::unique_ptr<LogBlockGeneric> qErrorLog;
  std::unique_ptr<LogBlockGeneric> angVelErrorLog;
  std::unique_ptr<LogBlockGeneric> posErrorLog;
  std::unique_ptr<LogBlockGeneric> velErrorLog;
  std::unique_ptr<LogBlockGeneric> qLog;
  std::unique_ptr<LogBlockGeneric> posLog;

 public:
  Eigen::Quaterniond last_q_error;
  InternalPose _pose;
  std::shared_ptr<Core::Publisher<ControlMetrics>> _metrics_publisher;
  void marker_cb(const Core::IncomingMessage<TrackingMarker>&);
  void publish_odom();
  CrazyBridge(Core::ArgumentParser);
  void run();
};

void onLogPosError(uint32_t time_in_ms, std::vector<double>* values,
                   void* /*userData*/);
void onLogVelError(uint32_t time_in_ms, std::vector<double>* values,
                   void* /*userData*/);
void onLogQError(uint32_t time_in_ms, std::vector<double>* values,
                 void* /*userData*/);
void onLogAngVelError(uint32_t time_in_ms, std::vector<double>* values,
                      void* /*userData*/);
void onLogQuartenion(uint32_t time_in_ms, std::vector<double>* values,
                     void* /*userData*/);
void onLogPosition(uint32_t time_in_ms, std::vector<double>* values,
                   void* /*userData*/);

#endif
