#ifndef REGISTRY_HPP
#define REGISTRY_HPP

#include <map>
#include <optional>
#include <string>
#include <time.h>
#include <vector>
#include <zmq.hpp>
#include <zmq_addon.hpp>

namespace Core {
struct Endpoint {
  std::string host;
  uint16_t port;
};

struct EndpointMetadata {
  Endpoint endpoint;
  time_t created_at;
  std::uint8_t ttl;
};

struct RegistryConfiguration {
  int16_t port;
};

class Registry {
 public:
  explicit Registry(RegistryConfiguration config);

  void run();
  void register_topic(const std::string topic, const Endpoint& host);

  void deregister_topic(const std::string topic);

  std::optional<Endpoint> check_topic(const std::string topic);
  void persist_topics();

  void handle_request();

 private:
  RegistryConfiguration config;
  zmq::context_t ctx;
  zmq::socket_t router;
  std::string _persistance_file;
  std::map<std::string, Endpoint> _topic_to_endpoint;
  std::map<std::string, std::vector<Endpoint>> _topic_to_waiters;
};
}  // namespace Core

#endif
