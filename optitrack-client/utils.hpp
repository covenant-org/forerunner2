#pragma once
#include <NatNetCAPI.h>
#include <NatNetClient.h>
#include <NatNetTypes.h>
#include <cstring>
#include <map>
#include <memory>
#include <stdlib.h>
#include <string>

void NATNET_CALLCONV MessageHandler(
    Verbosity msgType, const char* msg);  // receives NatNet error messages

int ConnectClient(NatNetClient* g_pClient,
                  sNatNetClientConnectParams& g_connectParams);

void UpdateDataToDescriptionMaps(
    sDataDescriptions* pDataDefs,
    std::map<int, int>& g_AssetIDtoAssetDescriptionOrder,
    std::map<int, std::string> g_AssetIDtoAssetName);

bool UpdateDataDescriptions(
    NatNetClient* g_pClient, sDataDescriptions* g_pDataDefs,
    std::map<int, int>& g_AssetIDtoAssetDescriptionOrder,
    std::map<int, std::string> g_AssetIDtoAssetName);

typedef struct MocapFrameWrapper {
  std::shared_ptr<sFrameOfMocapData> data;
  double transitLatencyMillisec;
  double clientLatencyMillisec;
} MocapFrameWrapper;

void printfBits(uint64_t val, int nBits);
