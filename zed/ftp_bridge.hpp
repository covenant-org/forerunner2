#ifndef FTP_BRIDGE_HPP
#define FTP_BRIDGE_HPP

#include "publisher.hpp"
#include "utils.hpp"
#include "vertex.hpp"
#include <capnp_schemas/generics.capnp.h>
#include <capnp_schemas/zed.capnp.h>
#include <capnp_schemas/sensors.capnp.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/ftp/ftp.h>
#include <memory>

class FtpBridge : public Core::Vertex {
 private:
  std::shared_ptr<Core::Subscriber<KeyValue>> _ftp_config_sub;
  std::shared_ptr<Core::Subscriber<PointCloudChunk>> _map_sub;
  std::shared_ptr<Core::Publisher<PointCloudChunk>> _map_pub;
  std::optional<std::shared_ptr<mavsdk::System>> _system = std::nullopt;
  std::shared_ptr<mavsdk::Ftp> _ftp_client = nullptr;
  mavsdk::Mavsdk _mavsdk;
  std::string _ftp_folder;
  std::string _tmp_folder;
  Core::WorkLock lock;

 public:
  FtpBridge(const Core::ArgumentParser&);
  void config_cb(const Core::IncomingMessage<KeyValue>&);
  void map_cb(const Core::IncomingMessage<PointCloudChunk>&);
  void poll_files();
  void run();
};

#endif  // !DEBUG
