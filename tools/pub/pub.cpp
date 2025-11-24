#include "argument_parser.hpp"
#include "capnp_schemas/envelope.capnp.h"
#include "publisher.hpp"

#include <capnp/compat/json.h>
#include <capnp/dynamic.h>
#include <capnp/schema-parser.h>

#include <kj/array.h>
#include <kj/exception.h>
#include <kj/filesystem.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>

#include <cstring>
#include <iomanip>
#include <iostream>

#include <optional>
#include <sstream>
#include <stdexcept>

#include <string>
#include <utility>
#include <vector>

#include <thread>

namespace {

constexpr const char* kDefaultRegistryUri = "tcp://127.0.0.1:4020";

class CliParser : public Core::ArgumentParser {
 public:
	CliParser(int argc, char** argv) : Core::ArgumentParser(argc, argv) {}
	void finalize() { this->parse(); }
};

std::string json_escape(const std::string& input) {
	std::ostringstream out;
	out << '"';

	for (unsigned char ch : input) {
		switch (ch) {
			case '\"':
				out << "\\\"";
				break;
			case '\\':
				out << "\\\\";
				break;
			case '\b':
				out << "\\b";
				break;
			case '\f':
				out << "\\f";
				break;
			case '\n':
				out << "\\n";
				break;
			case '\r':
				out << "\\r";
				break;
			case '\t':
				out << "\\t";
				break;
			default:
				if (ch < 0x20) {
					out << "\\u" << std::hex << std::uppercase << std::setw(4)
						<< std::setfill('0') << static_cast<int>(ch);
				} else {
					out << static_cast<char>(ch);
				}
		}
	}
	out << '"';
	return out.str();
}

bool case_equal(const std::string& lhs, const std::string& rhs) {
	return std::equal(
		lhs.begin(), lhs.end(), rhs.begin(), rhs.end(),
		[](unsigned char a, unsigned char b) {
			return std::tolower(static_cast<unsigned char>(a)) ==
				   std::tolower(static_cast<unsigned char>(b));
		});
}

std::string scalar_to_json(const YAML::Node& node) {
	const std::string tag = node.Tag();
	const std::string raw = node.Scalar();

	if (tag == "!!str") {
		return json_escape(raw);
	}

	if (raw.empty()) {
		return "\"\"";
	}

	if (raw == "~" || case_equal(raw, "null")) {
		return "null";
	}
	if (case_equal(raw, "true")) {
		return "true";
	}
	if (case_equal(raw, "false")) {
		return "false";
	}

	try {
		size_t consumed = 0;
		long long value = std::stoll(raw, &consumed, 0);
		if (consumed == raw.size()) {
			return std::to_string(value);
		}
	} catch (const std::exception&) {
		// Fall through to floating-point or string literal cases.
	}

	try {
		size_t consumed = 0;
		double value = std::stod(raw, &consumed);
		if (consumed == raw.size()) {
			std::ostringstream ss;
			ss << value;
			return ss.str();
		}
	} catch (const std::exception&) {
		// Fall back to string literal.
	}

	return json_escape(raw);
}

void write_json(const YAML::Node& node, std::ostringstream& out) {
	if (!node || node.IsNull()) {
		out << "null";
		return;
	}

	if (node.IsScalar()) {
		out << scalar_to_json(node);
		return;
	}

	if (node.IsSequence()) {
		out << '[';
		bool first = true;
		for (const auto& item : node) {
			if (!first) out << ',';
			write_json(item, out);
			first = false;
		}
		out << ']';
		return;
	}

	if (node.IsMap()) {
		out << '{';
		bool first = true;
		for (const auto& entry : node) {
			if (!first) out << ',';
			const std::string key = entry.first.as<std::string>();
			out << json_escape(key) << ':';
			write_json(entry.second, out);
			first = false;
		}
		out << '}';
		return;
	}

	out << "null";
}

std::string yaml_to_json(const std::string& text) {
	YAML::Node node = YAML::Load(text);
	std::ostringstream out;
	write_json(node, out);
	return out.str();
}

struct TypeInfo {
	std::string schema_path;
	std::string qualified_type;
	std::string leaf_name;
};

TypeInfo parse_type(const std::string& type_string,
					const std::optional<std::string>& schema_override) {
	std::string normalized = type_string;
	size_t pos = normalized.find("::");
	while (pos != std::string::npos) {
		normalized.replace(pos, 2, ".");
		pos = normalized.find("::", pos);
	}

	const auto dot = normalized.find('.');
	if (dot == std::string::npos || dot + 1 >= normalized.size()) {
		throw std::runtime_error("type must be in the form 'schema.Type'");
	}

	const std::string schema_part = normalized.substr(0, dot);
	const std::string type_part = normalized.substr(dot + 1);

	std::string schema_path;
	if (schema_override.has_value()) {
		schema_path = *schema_override;
	} else {
		schema_path = "messages/schemas/" + schema_part + ".capnp";
	}

	std::string leaf = type_part;
	auto last_dot = leaf.find_last_of('.');
	if (last_dot != std::string::npos && last_dot + 1 < leaf.size()) {
		leaf = leaf.substr(last_dot + 1);
	}

	return TypeInfo{schema_path, type_part, leaf};
}

struct LoadedSchema {
	capnp::StructSchema schema;
	std::string resolved_path;
};

LoadedSchema load_schema(capnp::SchemaParser& parser,
						 const kj::Directory& base_dir,
						 std::string schema_path,
						 const std::string& qualified_type) {
	std::vector<std::string> candidates;
	candidates.push_back(std::move(schema_path));
	if (candidates.front().rfind("messages/schemas/", 0) != 0) {
		candidates.push_back("messages/schemas/" + candidates.front());
	}

	for (size_t idx = 0; idx < candidates.size(); ++idx) {
		const bool last = idx + 1 == candidates.size();
		try {
			kj::ArrayPtr<const kj::ReadableDirectory* const> import_dirs;
			auto parsed = parser.parseFromDirectory(
				base_dir, kj::Path::parse(candidates[idx].c_str()), import_dirs);

			capnp::ParsedSchema current = parsed;
			size_t start = 0;
			while (start < qualified_type.size()) {
				const auto dot = qualified_type.find('.', start);
				const auto len =
					dot == std::string::npos ? std::string::npos : dot - start;
				const std::string part = qualified_type.substr(start, len);
				if (part.empty()) {
					throw std::runtime_error("invalid type name: '" + qualified_type + "'");
				}
				current = current.getNested(part.c_str());
				if (dot == std::string::npos) break;
				start = dot + 1;
			}

			LoadedSchema result;
			result.schema = current.asStruct();
			result.resolved_path = candidates[idx];
			return result;
		} catch (const kj::Exception& ex) {
			if (last) {
				throw std::runtime_error(std::string("Failed to load schema '") +
										 candidates[idx] + "': " +
										 ex.getDescription().cStr());
			}
		}
	}

	throw std::runtime_error("unable to resolve schema");
}

}  // namespace

int main(int argc, char** argv) {
	try {
		CliParser parser(argc, argv);
		parser.add_argument("topic").help("Topic name");
		parser.add_argument("type").help("Type in the form schema.Type");
		parser.add_argument("payload").help("YAML or JSON payload");
		parser.add_argument("--schema").help("Override schema path");
		parser.add_argument("--registry-uri")
			.default_value(std::string{kDefaultRegistryUri})
			.help("Registry URI");
		parser.add_argument("--allow-unknown")
			.help("Allow unknown fields in the payload")
			.default_value(false)
			.implicit_value(true)
			.flag();
		parser.add_argument("--count")
			.help("Number of messages to publish (0 for infinite)")
			.default_value(1)
			.scan<'i', int>();
		parser.add_argument("--rate")
			.help("Publish rate in Hz (0 for as fast as possible)")
			.default_value(0.0)
			.scan<'g', double>();

		parser.finalize();

		const std::string registry_uri = parser.get_argument<std::string>("--registry-uri");
		const std::string topic = parser.get_argument<std::string>("topic");
		const std::string type = parser.get_argument<std::string>("type");
		const std::string payload = parser.get_argument<std::string>("payload");
		const auto schema_override = parser.present("--schema");
		const bool allow_unknown = parser.get_argument<bool>("--allow-unknown");
		const int requested_count = parser.get_argument<int>("--count");
		const double rate_hz = parser.get_argument<double>("--rate");

		if (requested_count < 0) {
			throw std::runtime_error("--count must be non-negative");
		}
		if (!(std::isfinite(rate_hz)) || rate_hz < 0.0) {
			throw std::runtime_error("--rate must be finite and non-negative");
		}

		const TypeInfo info = parse_type(type, schema_override);

		capnp::SchemaParser schema_parser;
		kj::Own<kj::Filesystem> filesystem = kj::newDiskFilesystem();
		kj::Own<const kj::Directory> base_dir = filesystem->getCurrent().clone();

		const LoadedSchema loaded = load_schema(schema_parser, *base_dir, info.schema_path,
												info.qualified_type);

		capnp::JsonCodec codec;
		codec.setRejectUnknownFields(!allow_unknown);

		const std::string json_payload = yaml_to_json(payload);
		kj::ArrayPtr<const char> json_view(json_payload.data(), json_payload.size());

		Core::Publisher<Envelope> publisher(topic);
		publisher.setup(registry_uri);

		const bool infinite_loop = requested_count == 0;
		const bool throttled = rate_hz > 0.0;
		auto interval = std::chrono::steady_clock::duration::zero();
		if (throttled) {
			auto period = std::chrono::duration<double>(1.0 / rate_hz);
			interval = std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
			if (interval.count() <= 0) {
				interval = std::chrono::microseconds(1);
			}
		}

		auto next_deadline = std::chrono::steady_clock::now();
		std::int64_t published = 0;

		while (infinite_loop || published < requested_count) {
			capnp::MallocMessageBuilder message;
			auto envelope = message.initRoot<Envelope>();
			const uint64_t micros =
				std::chrono::duration_cast<std::chrono::microseconds>(
					std::chrono::system_clock::now().time_since_epoch())
					.count();
			envelope.setTimestampUsec(micros);
			envelope.setTopic(topic);
			envelope.setTypeId(loaded.schema.getProto().getId());
			envelope.setTypeName(info.leaf_name);
			envelope.setSchemaPath(loaded.resolved_path);

			auto payload_builder = envelope.initPayload();
			auto dynamic_builder = payload_builder.initAs<capnp::DynamicStruct>(loaded.schema);
			codec.decode(json_view, dynamic_builder);

			publisher.publish(message);
			++published;
			std::cout << "pub: published message on '" << topic << "' (#" << published
				  << ")" << std::endl;

			if (!throttled || (!infinite_loop && published >= requested_count)) {
				continue;
			}

			next_deadline += interval;
			std::this_thread::sleep_until(next_deadline);
		}

		std::cout << "pub: completed publishing " << published << " message(s) on '"
			  << topic << "'" << std::endl;
		return 0;
	} catch (const kj::Exception& ex) {
		std::cerr << "pub: " << ex.getDescription().cStr() << std::endl;
	} catch (const std::exception& ex) {
		std::cerr << "pub: " << ex.what() << std::endl;
	}
	return 1;
}
