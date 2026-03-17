#include "NatNetClient.h"
#include "NatNetTypes.h"
#include "utils.hpp"
#include <stdio.h>

void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char* msg) {
  // Optional: Filter out debug messages
  if (msgType < Verbosity_Info) {
    return;
  }

  printf("\n[NatNetLib]");

  switch (msgType) {
    case Verbosity_Debug:
      printf(" [DEBUG]");
      break;
    case Verbosity_Info:
      printf("  [INFO]");
      break;
    case Verbosity_Warning:
      printf("  [WARN]");
      break;
    case Verbosity_Error:
      printf(" [ERROR]");
      break;
    default:
      printf(" [?????]");
      break;
  }

  printf(": %s\n", msg);
}

int ConnectClient(NatNetClient* g_pClient,
                  sNatNetClientConnectParams& g_connectParams) {
  // Disconnect from any previous server (if connected)
  g_pClient->Disconnect();

  // Connect to NatNet server (e.g. Motive)
  int retCode = g_pClient->Connect(g_connectParams);
  if (retCode != ErrorCode_OK) {
    // Connection failed - print connection error code
    printf(
        "[SampleClinet] Unable to connect to server.  Error code: %d. "
        "Exiting.\n",
        retCode);
    return ErrorCode_Internal;
  } else {
    // Connection succeeded
    void* pResult;
    int nBytes = 0;
    ErrorCode ret = ErrorCode_OK;
    sServerDescription g_serverDescription;

    // example : print server info
    memset(&g_serverDescription, 0, sizeof(g_serverDescription));
    ret = g_pClient->GetServerDescription(&g_serverDescription);
    if (ret != ErrorCode_OK || !g_serverDescription.HostPresent) {
      printf(
          "[SampleClient] Unable to connect to server. Host not present. "
          "Exiting.\n");
      return 1;
    }
    printf("\n[SampleClient] Server application info:\n");
    printf("Application: %s (ver. %d.%d.%d.%d)\n",
           g_serverDescription.szHostApp, g_serverDescription.HostAppVersion[0],
           g_serverDescription.HostAppVersion[1],
           g_serverDescription.HostAppVersion[2],
           g_serverDescription.HostAppVersion[3]);
    printf("NatNet Version: %d.%d.%d.%d\n",
           g_serverDescription.NatNetVersion[0],
           g_serverDescription.NatNetVersion[1],
           g_serverDescription.NatNetVersion[2],
           g_serverDescription.NatNetVersion[3]);
    printf("Client IP:%s\n", g_connectParams.localAddress);
    printf("Server IP:%s\n", g_connectParams.serverAddress);
    printf("Server Name:%s\n", g_serverDescription.szHostComputerName);

    // example : get mocap frame rate
    ret = g_pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
    if (ret == ErrorCode_OK) {
      float fRate = *((float*)pResult);
      printf("Mocap Framerate : %3.2f\n", fRate);
    } else {
      printf("Error getting frame rate.\n");
    }
  }

  return ErrorCode_OK;
}

bool UpdateDataDescriptions(
    NatNetClient* g_pClient, sDataDescriptions* g_pDataDefs,
    std::map<int, int>& g_AssetIDtoAssetDescriptionOrder,
    std::map<int, std::string> g_AssetIDtoAssetName) {
  // release memory allocated by previous in previous GetDataDescriptionList()
  if (g_pDataDefs) {
    NatNet_FreeDescriptions(g_pDataDefs);
  }

  // Retrieve Data Descriptions from Motive
  printf("\n\n[SampleClient] Requesting Data Descriptions...\n");
  int iResult = g_pClient->GetDataDescriptionList(&g_pDataDefs);
  if (iResult != ErrorCode_OK || g_pDataDefs == NULL) {
    return false;
  }
  UpdateDataToDescriptionMaps(g_pDataDefs, g_AssetIDtoAssetDescriptionOrder,
                              g_AssetIDtoAssetName);

  return true;
}

/**
 * Update maps whenever the asset list in Motive has changed (as indicated in
 * the data packet's TrackedModelsChanged bit)
 *
 * \param pDataDefs
 */
void UpdateDataToDescriptionMaps(
    sDataDescriptions* pDataDefs,
    std::map<int, int>& g_AssetIDtoAssetDescriptionOrder,
    std::map<int, std::string> g_AssetIDtoAssetName) {
  g_AssetIDtoAssetDescriptionOrder.clear();
  g_AssetIDtoAssetName.clear();
  int assetID = 0;
  std::string assetName = "";
  int index = 0;
  int cameraIndex = 0;

  if (pDataDefs == nullptr || pDataDefs->nDataDescriptions <= 0) return;

  for (int i = 0; i < pDataDefs->nDataDescriptions; i++) {
    assetID = -1;
    assetName = "";

    if (pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody) {
      sRigidBodyDescription* pRB =
          pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
      assetID = pRB->ID;
      assetName = std::string(pRB->szName);
    } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Skeleton) {
      sSkeletonDescription* pSK =
          pDataDefs->arrDataDescriptions[i].Data.SkeletonDescription;
      assetID = pSK->skeletonID;
      assetName = std::string(pSK->szName);

    } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet) {
      // Skip markersets for now as they dont have unique id's, but do increase
      // the index as they are in the data packet
      index++;
      continue;
      /*
      sMarkerSetDescription* pDesc =
      pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription; assetID =
      index; assetName = pDesc->szName;
      */
    }

    else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_ForcePlate) {
      sForcePlateDescription* pDesc =
          pDataDefs->arrDataDescriptions[i].Data.ForcePlateDescription;
      assetID = pDesc->ID;
      assetName = pDesc->strSerialNo;
    } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Device) {
      sDeviceDescription* pDesc =
          pDataDefs->arrDataDescriptions[i].Data.DeviceDescription;
      assetID = pDesc->ID;
      assetName = std::string(pDesc->strName);
    } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Camera) {
      // skip cameras as they are not in the data packet
      continue;
    } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Asset) {
      sAssetDescription* pDesc =
          pDataDefs->arrDataDescriptions[i].Data.AssetDescription;
      assetID = pDesc->AssetID;
      assetName = std::string(pDesc->szName);
    }

    if (assetID == -1) {
      printf(
          "\n[SampleClient] Warning : Unknown data type in description list "
          ": %d\n",
          pDataDefs->arrDataDescriptions[i].type);
    } else {
      // Add to Asset ID to Asset Name map
      std::pair<std::map<int, std::string>::iterator, bool> insertResult;
      insertResult = g_AssetIDtoAssetName.insert(
          std::pair<int, std::string>(assetID, assetName));
      if (insertResult.second == false) {
        printf(
            "\n[SampleClient] Warning : Duplicate asset ID already in Name "
            "map (Existing:%d,%s\tNew:%d,%s\n)",
            insertResult.first->first, insertResult.first->second.c_str(),
            assetID, assetName.c_str());
      }
    }

    // Add to Asset ID to Asset Description Order map
    if (assetID != -1) {
      std::pair<std::map<int, int>::iterator, bool> insertResult;
      insertResult = g_AssetIDtoAssetDescriptionOrder.insert(
          std::pair<int, int>(assetID, index++));
      if (insertResult.second == false) {
        printf(
            "\n[SampleClient] Warning : Duplicate asset ID already in Order "
            "map (ID:%d\tOrder:%d\n)",
            insertResult.first->first, insertResult.first->second);
      }
    }
  }
}

void printfBits(uint64_t val, int nBits) {
  for (int i = nBits - 1; i >= 0; i--) {
    printf("%d", (int)(val >> i) & 0x01);
  }
  printf("\n");
}
