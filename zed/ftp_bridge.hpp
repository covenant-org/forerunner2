#ifndef FTP_BRIDGE_HPP
#define FTP_BRIDGE_HPP

#include "core.h"
#include <capnp_schemas/generics.capnp.h>
#include <capnp_schemas/zed.capnp.h>

class FtpBridge : Core::Vertex {
 private:
  std::shared_ptr<Core::Subscriber<KeyValue>> _ftp_config_sub;
  std::shared_ptr<Core::Subscriber<PointCloudChunk>> _map_sub;

 public:
  FtpBridge(const Core::ArgumentParser&);
  void config_cb(const Core::IncomingMessage<KeyValue>);
  void map_cb(const Core::IncomingMessage<PointCloudChunk>);
  void run();
};

#endif  // !DEBUG
