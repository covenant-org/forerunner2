#include "comprobation.hpp"

#include <netdb.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstdlib>
#include <iostream>
#include <thread>

namespace Rerun::Service {

ConnectivityProbe::ConnectivityProbe(Core::ArgumentParser parser)
		: Core::Vertex(parser), _online(false), _registry_running(false),
			_watch_mode(this->get_argument<bool>("--watch")) {
	// TODO: Hacer que revise en todas las interfaces, por si tiene distintas formas de acceso a internet
	_host = "8.8.8.8";
	_port = 53;
	_timeout = std::chrono::milliseconds(1500);
	_interval = std::chrono::milliseconds(5000);
}

bool ConnectivityProbe::check_connectivity() {
	addrinfo hints{};
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	addrinfo* result = nullptr;
	auto service = std::to_string(_port);
	auto lookup = ::getaddrinfo(_host.c_str(), service.c_str(), &hints, &result);
	if (lookup != 0 || result == nullptr) {
		this->_logger.debug("getaddrinfo failed for %s:%s", _host.c_str(),
												gai_strerror(lookup));
		if (result != nullptr) ::freeaddrinfo(result);
		return false;
	}

	bool reachable = false;

	for (addrinfo* entry = result; entry != nullptr && !reachable;
			 entry = entry->ai_next) {
		int fd = ::socket(entry->ai_family, entry->ai_socktype, entry->ai_protocol);
		if (fd < 0) continue;

		const auto millis = static_cast<long>(_timeout.count());
		timeval tv{};
		tv.tv_sec = millis / 1000;
		tv.tv_usec = (millis % 1000) * 1000;
		::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
		::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

		if (::connect(fd, entry->ai_addr, entry->ai_addrlen) == 0) {
			reachable = true;
		}

		::close(fd);
	}

	::freeaddrinfo(result);
	return reachable;
}

bool ConnectivityProbe::check_registry_process() {
	int result = std::system("pgrep -x registry >/dev/null 2>&1");
	if (result == -1) {
		return false;
	}
	return WIFEXITED(result) && WEXITSTATUS(result) == 0;
}

void ConnectivityProbe::run() {
	this->_logger.info("Checking connectivity against %s:%u every %lld ms",
										 _host.c_str(), static_cast<unsigned>(_port),
										 static_cast<long long>(_interval.count()));

	if (!_watch_mode) {
		const bool internet_ok = check_connectivity();
		const bool registry_ok = check_registry_process();

		if (internet_ok) {
			this->_logger.info("Internet reachable via %s:%u", _host.c_str(),
							 static_cast<unsigned>(_port));
		} else {
			this->_logger.warn("Connectivity lost towards %s:%u", _host.c_str(),
							 static_cast<unsigned>(_port));
		}

		if (registry_ok) {
			this->_logger.warn("Registry process detected");
		} else {
			this->_logger.info("Registry process not found");
		}

		std::cout << "internet=" << (internet_ok ? "true" : "false")
				  << " registry=" << (!registry_ok ? "true" : "false")
				  << std::endl;
		return;
	}

	_registry_running = check_registry_process();
	if (_registry_running) {
		this->_logger.warn("Registry process detected");
	} else {
		this->_logger.info("Registry process not found");
	}

	while (true) {
		const bool online = check_connectivity();

		if (online != _online) {
			_online = online;
			if (online) {
				this->_logger.info("Internet reachable via %s:%u", _host.c_str(),
													 static_cast<unsigned>(_port));
			} else {
				this->_logger.warn("Connectivity lost towards %s:%u", _host.c_str(),
								 static_cast<unsigned>(_port));
			}
		}

		const bool registry_running = check_registry_process();
		if (registry_running != _registry_running) {
			_registry_running = registry_running;
			if (registry_running) {
				this->_logger.warn("Registry process detected");
			} else {
				this->_logger.info("Registry process not found");
			}
		}

		std::this_thread::sleep_for(_interval);
	}
}

}  // namespace Rerun::Service

int main(int argc, char** argv) {
	Core::BaseArgumentParser parser(argc, argv);
	parser.add_argument("--watch")
			.default_value(false)
			.implicit_value(true)
			.help("Keep monitoring connectivity and registry status")
			.flag();

	Rerun::Service::ConnectivityProbe probe(parser);
	probe.run();
	return 0;
}
