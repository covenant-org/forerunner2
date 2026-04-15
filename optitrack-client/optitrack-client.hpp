#pragma once
#include "argument_parser.hpp"
#include "capnp_schemas/optitrack.capnp.h"
#include "publisher.hpp"
#include "utils.hpp"
#include "vertex.hpp"
#include <NatNetCAPI.h>
#include <NatNetClient.h>
#include <NatNetTypes.h>
#include <deque>
#include <inttypes.h>
#include <memory>
#include <mutex>
#include <stdio.h>
#include <stdlib.h>

class OptitrackClient : public Core::Vertex {
 private:
  std::shared_ptr<Core::Publisher<TrackingMarker>> _marker_publisher;
  std::shared_ptr<Core::Publisher<RigidBody>> _rigid_publisher;
  NatNetClient* g_pClient = NULL;
  bool gUpdatedDataDescriptions = false;
  bool gNeedUpdatedDataDescriptions = true;
  sDataDescriptions* g_pDataDefs = NULL;
  std::map<int, int> g_AssetIDtoAssetDescriptionOrder;
  std::map<int, std::string> g_AssetIDtoAssetName;
  void OutputFrameQueueToConsole();

 public:
  const int kMaxQueueSize = 500;
  std::timed_mutex gNetworkQueueMutex;
  std::deque<MocapFrameWrapper> gNetworkQueue;
  explicit OptitrackClient(Core::ArgumentParser);
  NatNetClient* getNatNetClient() { return this->g_pClient; }
  void run();
};
