#include "argument_parser.hpp"
#include "capnp_schemas/registry.capnp.h"
#include "vertex.hpp"
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

class TopicsTool : public Core::Vertex {
 public:
	explicit TopicsTool(Core::ArgumentParser args)
			: Core::Vertex(args) {
		auto topic = this->_args.get_argument("--topic");
		_client = this->create_action_client<TopicsListRequest, TopicsListResponse>(
				topic);
	}

	void run() override {
		try {
			const bool include_internal =
					this->_args.get_argument<bool>("--include-internal");
			auto request = _client->new_msg();
			request.content.setIncludeInternal(include_internal);
			auto response = request.send();
			if (!response.has_value()) {
				std::cerr << "No response from registry topics action" << std::endl;
				return;
			}

			auto topics = response->content.getTopics();
			if (topics.size() == 0) {
				std::cout << "No topics registered" << std::endl;
				return;
			}

			std::cout << std::left;
			std::cout << std::setw(40) << "Topic" << std::setw(20) << "Host"
								<< "Port" << std::endl;
			std::cout << std::string(40 + 20 + 6, '-') << std::endl;

			for (auto topic_entry : topics) {
				const auto name = topic_entry.getName();
				const auto host = topic_entry.getHost();
				std::cout << std::setw(40) << name.cStr() << std::setw(20)
									<< host.cStr() << topic_entry.getPort() << std::endl;
			}
		} catch (const std::exception& ex) {
			std::cerr << "Failed to query topics: " << ex.what() << std::endl;
		}
	}

 private:
	std::shared_ptr<Core::ActionClient<TopicsListRequest, TopicsListResponse>>
			_client;
};

int main(int argc, char** argv) {
	Core::BaseArgumentParser args(argc, argv);
	args.add_argument("--topic").default_value("registry/topics");
	args.add_argument("--include-internal")
			.help("Include registry-internal topics in the output")
			.default_value(false)
			.implicit_value(true)
			.flag();

	TopicsTool tool(args);
	tool.run();
	return 0;
}
