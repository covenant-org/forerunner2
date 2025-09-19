#ifndef FTP_BRIDGE_HPP
#define FTP_BRIDGE_HPP

#include "publisher.hpp"
#include "utils.hpp"
#include "vertex.hpp"
#include "../messages/build/generated/sensors.capnp.h"
#include "../messages/build/generated/generics.capnp.h"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/ftp/ftp.h>
#include <memory>

class FtpBridgePD : public Core::Vertex {
 private:
  std::shared_ptr<Core::Subscriber<KeyValue>> _ftp_config_sub;
  std::shared_ptr<Core::Subscriber<ImageData>> _people_image_sub;
  std::shared_ptr<Core::Publisher<ImageData>> _people_image_pub ;
  std::optional<std::shared_ptr<mavsdk::System>> _system = std::nullopt;
  std::shared_ptr<mavsdk::Ftp> _ftp_client = nullptr;
  mavsdk::Mavsdk _mavsdk;
  std::string _ftp_folder;
  std::string _tmp_folder;
  Core::WorkLock lock;

 public:
  FtpBridgePD(const Core::ArgumentParser&);
  void config_cb(const Core::IncomingMessage<KeyValue>&);
  void people_image_cb(const Core::IncomingMessage<ImageData>&);
  void poll_files();
  void run();
};

#endif  // !DEBUG
