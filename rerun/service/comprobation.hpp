#pragma once

#include "vertex.hpp"

#include <chrono>
#include <cstdint>
#include <string>

namespace Rerun::Service {

class ConnectivityProbe : public Core::Vertex {
 public:
	explicit ConnectivityProbe(Core::ArgumentParser parser);

	void run() override;

 private:
	bool check_connectivity();
	bool check_registry_process();

	std::string _host;
	uint16_t _port;
	std::chrono::milliseconds _timeout;
	std::chrono::milliseconds _interval;
	bool _online;
	bool _registry_running;
	bool _watch_mode;
};

}  // namespace Rerun::Service
