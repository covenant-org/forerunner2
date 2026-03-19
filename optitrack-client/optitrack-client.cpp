#include "NatNetTypes.h"
#include "argument_parser.hpp"
#include "optitrack-client.hpp"
#include <stdexcept>

void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData) {
  OptitrackClient* oClient = (OptitrackClient*)pUserData;
  if (!oClient) return;
  NatNetClient* pClient = oClient->getNatNetClient();
  if (!pClient) return;

  // Note : This function is called every 1 / mocap rate ( e.g. 100 fps = every
  // 10 msecs ) We don't want to do too much here and cause the network
  // processing thread to get behind, so let's just safely add this frame to our
  // shared  'network' frame queue and return.

  // Note : The 'data' ptr passed in is managed by NatNet and cannot be used
  // outside this function. Since we are keeping the data, we need to make a
  // copy of it.
  std::shared_ptr<sFrameOfMocapData> pDataCopy =
      std::make_shared<sFrameOfMocapData>();
  NatNet_CopyFrame(data, pDataCopy.get());

  MocapFrameWrapper f;
  f.data = pDataCopy;
  f.clientLatencyMillisec =
      pClient->SecondsSinceHostTimestamp(data->CameraMidExposureTimestamp) *
      1000.0;
  f.transitLatencyMillisec =
      pClient->SecondsSinceHostTimestamp(data->TransmitTimestamp) * 1000.0;

  if (oClient->gNetworkQueueMutex.try_lock_for(std::chrono::milliseconds(5))) {
    oClient->gNetworkQueue.push_back(f);

    // Maintain a cap on the queue size, removing oldest as necessary
    while ((int)oClient->gNetworkQueue.size() > oClient->kMaxQueueSize) {
      f = oClient->gNetworkQueue.front();
      NatNet_FreeFrame(f.data.get());
      oClient->gNetworkQueue.pop_front();
    }
    oClient->gNetworkQueueMutex.unlock();
  } else {
    // Unable to lock the frame queue and we chose not to wait - drop the frame
    // and notify
    NatNet_FreeFrame(pDataCopy.get());
    printf("\nFrame dropped (Frame : %d)\n", f.data->iFrame);
  }

  return;
}

OptitrackClient::OptitrackClient(Core::ArgumentParser args)
    : Core::Vertex(args) {
  this->_pose_publisher =
      this->create_publisher<PoseStamped>("optitrack/marker");
  // Print NatNet client version info
  unsigned char ver[4];
  NatNet_GetVersion(ver);
  printf("NatNet Custom Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1],
         ver[2], ver[3]);

  NatNet_SetLogCallback(MessageHandler);
  g_pClient = new NatNetClient();
  g_pClient->SetFrameReceivedCallback(
      DataHandler,
      this);  // this function will receive data from the server
}

void OptitrackClient::run() {
  // Get info for, at most, the first 10 servers to respond.
  const int kMaxDescriptions = 10;
  sNatNetDiscoveredServer servers[kMaxDescriptions];
  int actualNumDescriptions = kMaxDescriptions;
  NatNet_BroadcastServerDiscovery(servers, &actualNumDescriptions);

  if (actualNumDescriptions == 0) {
    throw std::runtime_error("[Error] No NetNat servers found");
  }
  const sNatNetDiscoveredServer& discoveredServer = servers[0];

  char g_discoveredMulticastGroupAddr[kNatNetIpv4AddrStrLenMax] =
      NATNET_DEFAULT_MULTICAST_ADDRESS;
  sNatNetClientConnectParams g_connectParams;
  snprintf(g_discoveredMulticastGroupAddr,
           sizeof g_discoveredMulticastGroupAddr,
           "%" PRIu8 ".%" PRIu8 ".%" PRIu8 ".%" PRIu8 "",
           discoveredServer.serverDescription.ConnectionMulticastAddress[0],
           discoveredServer.serverDescription.ConnectionMulticastAddress[1],
           discoveredServer.serverDescription.ConnectionMulticastAddress[2],
           discoveredServer.serverDescription.ConnectionMulticastAddress[3]);

  g_connectParams.connectionType =
      discoveredServer.serverDescription.ConnectionMulticast
          ? ConnectionType_Multicast
          : ConnectionType_Unicast;
  g_connectParams.serverCommandPort = discoveredServer.serverCommandPort;
  g_connectParams.serverDataPort =
      discoveredServer.serverDescription.ConnectionDataPort;
  g_connectParams.serverAddress = discoveredServer.serverAddress;
  g_connectParams.localAddress = discoveredServer.localAddress;
  g_connectParams.multicastAddress = g_discoveredMulticastGroupAddr;

  printf("[Info] Connecting to %s\n", g_connectParams.serverAddress);

  int iResult = ConnectClient(g_pClient, g_connectParams);
  if (iResult != ErrorCode_OK) {
    throw std::runtime_error(
        "Error initializing client. See log for details. Exiting.\n");
  } else {
    printf("Client initialized and ready.\n");
  }

  gUpdatedDataDescriptions = UpdateDataDescriptions(
      g_pClient, g_pDataDefs, g_AssetIDtoAssetDescriptionOrder,
      g_AssetIDtoAssetName);
  if (!gUpdatedDataDescriptions) {
    printf(
        "[SampleClient] ERROR : Unable to retrieve Data Descriptions from "
        "Motive.\n");
  }

  printf(
      "\n[SampleClient] Client is connected to server and listening for "
      "data...\n");
  bool bRunning = true;
  while (bRunning) {
    // If Motive Asset list has changed, update our lookup maps
    if (gNeedUpdatedDataDescriptions) {
      gUpdatedDataDescriptions = UpdateDataDescriptions(
          g_pClient, g_pDataDefs, g_AssetIDtoAssetDescriptionOrder,
          g_AssetIDtoAssetName);
      if (gUpdatedDataDescriptions) {
        gNeedUpdatedDataDescriptions = false;
      }
    }

    // print all mocap frames in data queue to console
    OutputFrameQueueToConsole();

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

/**
 * Output frame queue to console.
 *
 */
void OptitrackClient::OutputFrameQueueToConsole() {
  // Add data from the network queue into our display queue in order to quickly
  // free up access to the network queue.
  std::deque<MocapFrameWrapper> displayQueue;
  if (gNetworkQueueMutex.try_lock_for(std::chrono::milliseconds(5))) {
    for (MocapFrameWrapper f : gNetworkQueue) {
      displayQueue.push_back(f);
    }

    // Release all frames in network queue
    gNetworkQueue.clear();
    gNetworkQueueMutex.unlock();
  }

  // Now we can take our time displaying our data without
  // worrying about interfering with the network processing queue.
  for (MocapFrameWrapper f : displayQueue) {
    sFrameOfMocapData* data = f.data.get();

    printf(
        "\n=====================  New Packet Arrived  "
        "=============================\n");
    printf("FrameID : %d\n", data->iFrame);
    printf("Timestamp : %3.2lf\n", data->fTimestamp);
    printf("Params : ");
    printfBits(data->params, sizeof(data->params) * 8);

    // timecode - for systems with an eSync and SMPTE timecode generator -
    // decode to values
    int hour, minute, second, frame, subframe;
    NatNet_DecodeTimecode(data->Timecode, data->TimecodeSubframe, &hour,
                          &minute, &second, &frame, &subframe);
    char szTimecode[128] = "";
    NatNet_TimecodeStringify(data->Timecode, data->TimecodeSubframe, szTimecode,
                             128);
    printf("Timecode : %s\n", szTimecode);

    const bool bSystemLatencyAvailable = data->CameraMidExposureTimestamp != 0;
    if (bSystemLatencyAvailable) {
      printf("NatNet Transit latency : %.2lf milliseconds\n",
             f.transitLatencyMillisec);

      // Total Client latency is defined as the sum of system latency and the
      // transit time taken to relay the data to the NatNet client. This is the
      // all-inclusive measurement (photons to client processing). You could
      // equivalently do the following (not accounting for time elapsed since we
      // calculated transit latency above):
      // const double clientLatencyMillisec = systemLatencyMillisec +
      // transitLatencyMillisec;
      printf("Total Client latency : %.2lf milliseconds \n",
             f.clientLatencyMillisec);
    } else {
      printf("Transit latency : %.2lf milliseconds\n",
             f.transitLatencyMillisec);
    }

    // precision timestamps (optionally present, typically PTP) (NatNet 4.1 and
    // later)
    if (data->PrecisionTimestampSecs != 0) {
      printf("Precision Timestamp Seconds : %d\n",
             data->PrecisionTimestampSecs);
      printf("Precision Timestamp Fractional Seconds : %d\n",
             data->PrecisionTimestampFractionalSecs);
    }

    if (gNeedUpdatedDataDescriptions) {
      printf("\n\n[SampleClient] Waiting for updated asset list\n");
      break;
    }

    bool bTrackedModelsChanged = ((data->params & 0x02) != 0);
    if (bTrackedModelsChanged) {
      printf(
          "\n\nMotive asset list changed.  Requesting new data "
          "descriptions.\n");
      gNeedUpdatedDataDescriptions = true;
      break;
    }

    bool bIsRecording = ((data->params & 0x01) != 0);
    if (bIsRecording) {
      printf("\nRECORDING\n");
    }

    // Rigid Bodies
    int i = 0;
    printf("------------------------\n");
    printf("Rigid Bodies [Count=%d]\n", data->nRigidBodies);
    for (i = 0; i < data->nRigidBodies; i++) {
      // params
      // 0x01 : bool, rigid body was successfully tracked in this frame
      bool bTrackingValid = data->RigidBodies[i].params & 0x01;
      int streamingID = data->RigidBodies[i].ID;
      auto msg = this->_pose_publisher->new_msg();
      auto pose = msg.content.initPose();
      auto orientation = pose.initOrientation();
      auto position = pose.initPosition();
      position.setX(data->RigidBodies[i].x);
      position.setY(data->RigidBodies[i].y);
      position.setZ(data->RigidBodies[i].z);
      orientation.setX(data->RigidBodies[i].qx);
      orientation.setY(data->RigidBodies[i].qy);
      orientation.setZ(data->RigidBodies[i].qz);
      orientation.setW(data->RigidBodies[i].qw);
      msg.publish();

      printf("%s [ID=%d  Error(mm)=%.5f  Tracked=%d]\n",
             g_AssetIDtoAssetName[streamingID].c_str(), streamingID,
             data->RigidBodies[i].MeanError * 1000.0f, bTrackingValid);
      printf("\tx\ty\tz\tqx\tqy\tqz\tqw\n");
      printf("\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
             data->RigidBodies[i].x, data->RigidBodies[i].y,
             data->RigidBodies[i].z, data->RigidBodies[i].qx,
             data->RigidBodies[i].qy, data->RigidBodies[i].qz,
             data->RigidBodies[i].qw);
    }

    // Skeletons
    printf("------------------------\n");
    printf("Skeletons [Count=%d]\n", data->nSkeletons);
    for (i = 0; i < data->nSkeletons; i++) {
      sSkeletonData skData = data->Skeletons[i];
      printf("Skeleton [ID=%d  Bone count=%d]\n", skData.skeletonID,
             skData.nRigidBodies);
      for (int j = 0; j < skData.nRigidBodies; j++) {
        sRigidBodyData rbData = skData.RigidBodyData[j];
        printf("Bone %d\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
               rbData.ID, rbData.x, rbData.y, rbData.z, rbData.qx, rbData.qy,
               rbData.qz, rbData.qw);
      }
    }

    // Trained Markerset Data (Motive 3.1 / NatNet 4.1 and later)
    printf("------------------------\n");
    printf("Assets [Count=%d]\n", data->nAssets);
    for (int i = 0; i < data->nAssets; i++) {
      sAssetData asset = data->Assets[i];
      printf("Trained Markerset [ID=%d  Bone count=%d   Marker count=%d]\n",
             asset.assetID, asset.nRigidBodies, asset.nMarkers);

      // Trained Markerset Rigid Bodies
      for (int j = 0; j < asset.nRigidBodies; j++) {
        // note : Trained markerset ids are of the form:
        // parent markerset ID  : high word (upper 16 bits of int)
        // rigid body id        : low word  (lower 16 bits of int)
        int assetID, rigidBodyID;
        sRigidBodyData rbData = asset.RigidBodyData[j];
        NatNet_DecodeID(rbData.ID, &assetID, &rigidBodyID);
        printf("Bone %d\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
               rigidBodyID, rbData.x, rbData.y, rbData.z, rbData.qx, rbData.qy,
               rbData.qz, rbData.qw);
      }

      // Trained Markerset markers
      for (int j = 0; j < asset.nMarkers; j++) {
        sMarker marker = asset.MarkerData[j];
        int assetID, markerID;
        NatNet_DecodeID(marker.ID, &assetID, &markerID);
        printf(
            "Marker [AssetID=%d, MarkerID=%d] [size=%3.2f] "
            "[pos=%3.2f,%3.2f,%3.2f] [residual(mm)=%.4f]\n",
            assetID, markerID, marker.size, marker.x, marker.y, marker.z,
            marker.residual * 1000.0f);
      }
    }

    // labeled markers - this includes all markers (Active, Passive, and
    // 'unlabeled' (markers with no asset but a PointCloud ID)
    bool bOccluded;      // marker was not visible (occluded) in this frame
    bool bPCSolved;      // reported position provided by point cloud solve
    bool bModelSolved;   // reported position provided by model solve
    bool bHasModel;      // marker has an associated asset in the data stream
    bool bUnlabeled;     // marker is 'unlabeled', but has a point cloud ID that
                         // matches Motive PointCloud ID (In Motive 3D View)
    bool bActiveMarker;  // marker is an actively labeled LED marker

    printf("------------------------\n");
    printf("Markers [Count=%d]\n", data->nLabeledMarkers);
    for (i = 0; i < data->nLabeledMarkers; i++) {
      bOccluded = ((data->LabeledMarkers[i].params & 0x01) != 0);
      bPCSolved = ((data->LabeledMarkers[i].params & 0x02) != 0);
      bModelSolved = ((data->LabeledMarkers[i].params & 0x04) != 0);
      bHasModel = ((data->LabeledMarkers[i].params & 0x08) != 0);
      bUnlabeled = ((data->LabeledMarkers[i].params & 0x10) != 0);
      bActiveMarker = ((data->LabeledMarkers[i].params & 0x20) != 0);

      sMarker marker = data->LabeledMarkers[i];

      // Marker ID Scheme:
      // Active Markers:
      //   ID = ActiveID, correlates to RB ActiveLabels list
      // Passive Markers:
      //   If Asset with Legacy Labels
      //      AssetID 	(Hi Word)
      //      MemberID	(Lo Word)
      //   Else
      //      PointCloud ID
      int modelID, markerID;
      NatNet_DecodeID(marker.ID, &modelID, &markerID);

      char szMarkerType[512];
      if (bActiveMarker)
        strcpy(szMarkerType, "Active");
      else if (bUnlabeled)
        strcpy(szMarkerType, "Unlabeled");
      else
        strcpy(szMarkerType, "Labeled");

      printf(
          "%s Marker [ModelID=%d, MarkerID=%d] [size=%3.2f] "
          "[pos=%3.2f,%3.2f,%3.2f] [residual(mm)=%.4f]\n",
          szMarkerType, modelID, markerID, marker.size, marker.x, marker.y,
          marker.z, marker.residual * 1000.0f);
    }
  }

  // Release all frames (and frame data) in the display queue
  for (MocapFrameWrapper f : displayQueue) {
    NatNet_FreeFrame(f.data.get());
  }
  displayQueue.clear();
}

int main(int argc, char** argv) {
  Core::BaseArgumentParser args(argc, argv);
  OptitrackClient node(args);
  node.run();
  return 0;
}
