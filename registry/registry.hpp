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

struct EndpointEfimeral {
  Endpoint endpoint;
  time_t created_at;
  std::uint8_t ttl;
};

class Registry {
  public:
  explicit Registry(int16_t port);

  void run();

  void register_topic(const std::string topic, const Endpoint& host);
  void deregister_topic(const std::string topic);

  std::optional<Endpoint> check_topic(const std::string topic);

  void persist_topics();

 private:
  int16_t port;
  zmq::context_t ctx;
  std::string _persistance_file;
  std::map<std::string, Endpoint> _topic_to_endpoint;
  std::map<std::string, std::vector<Endpoint>> _topic_to_waiters;
};
}  // namespace Core

#endif