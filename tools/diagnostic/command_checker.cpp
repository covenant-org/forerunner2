#include "command_checker.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <utility>

CommandChecker::CommandChecker(Core::Logger& logger,
                               std::string name,
                               std::string command,
                               std::vector<std::string> expected_markers,
                               std::string command_error_message,
                               std::string success_message,
                               std::string failure_message,
                               MatchMode match_mode,
                               bool case_sensitive)
    : BaseChecker(std::move(name), logger),
      command_(std::move(command)),
      markers_(std::move(expected_markers)),
      command_error_message_(std::move(command_error_message)),
      success_message_(std::move(success_message)),
      failure_message_(std::move(failure_message)),
      match_mode_(match_mode),
      case_sensitive_(case_sensitive) {
    markers_processed_.reserve(markers_.size());
    for (const auto& marker : markers_) {
        markers_processed_.push_back(case_sensitive_ ? marker : to_lower(marker));
    }
}

DiagnosticResult CommandChecker::run() {
    logger().debug("Running command: %s", command_.c_str());
    log_expected_patterns();

    CommandResult result = run_command(command_);
    log_command_debug(command_, result);

    if (result.exit_code != 0) {
        logger().debug("Command returned a non-zero exit code (%d)", result.exit_code);
        return {name(), false, command_error_message_};
    }

    std::vector<std::string> matched_lines;
    std::vector<std::string> missing_markers;
    const bool success = evaluate_output(result.output, matched_lines, missing_markers);

    if (success) {
        for (const auto& line : matched_lines) {
            logger().debug("Matched output line: %s", line.c_str());
        }
        logger().debug("%s passed the verification", name().c_str());
        return {name(), true, success_message_};
    }

    if (!missing_markers.empty()) {
        for (const auto& marker : missing_markers) {
            logger().debug("Missing pattern: %s", marker.c_str());
        }
    } else {
        logger().debug("No matches were found in the output");
    }

    logger().debug("%s failed the verification", name().c_str());
    return {name(), false, failure_message_};
}

bool CommandChecker::evaluate_output(const std::string& output,
                                     std::vector<std::string>& matched_lines,
                                     std::vector<std::string>& missing_markers) const {
    if (markers_.empty()) {
        logger().debug("No patterns specified; assuming success when the command executes correctly");
        return true;
    }

    std::vector<bool> found(markers_.size(), false);
    std::vector<std::string> matched_by_marker(markers_.size());

    std::istringstream stream(output);
    std::string line;
    while (std::getline(stream, line)) {
        std::string comparison = prepare_line(line);
        for (size_t i = 0; i < markers_processed_.size(); ++i) {
            if (found[i]) {
                continue;
            }
            if (comparison.find(markers_processed_[i]) != std::string::npos) {
                found[i] = true;
                matched_by_marker[i] = line;
                if (match_mode_ == MatchMode::Any) {
                    matched_lines.push_back(line);
                    return true;
                }
            }
        }
    }

    if (match_mode_ == MatchMode::Any) {
        for (size_t i = 0; i < found.size(); ++i) {
            if (found[i]) {
                matched_lines.push_back(matched_by_marker[i]);
            }
        }
        return !matched_lines.empty();
    }

    bool all_found = std::all_of(found.begin(), found.end(), [](bool value) { return value; });
    if (all_found) {
        for (const auto& line_match : matched_by_marker) {
            if (!line_match.empty()) {
                matched_lines.push_back(line_match);
            }
        }
        return true;
    }

    for (size_t i = 0; i < found.size(); ++i) {
        if (!found[i]) {
            missing_markers.push_back(markers_[i]);
        }
    }
    return false;
}

std::string CommandChecker::to_lower(const std::string& value) const {
    std::string transformed = value;
    std::transform(transformed.begin(), transformed.end(), transformed.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return transformed;
}

std::string CommandChecker::strip_ansi_sequences(const std::string& value) const {
    std::string result;
    result.reserve(value.size());

    for (size_t i = 0; i < value.size();) {
        unsigned char c = static_cast<unsigned char>(value[i]);
        if (c == 0x1B) {
            ++i;
            if (i < value.size() && value[i] == '[') {
                ++i;
                while (i < value.size() && value[i] < '@') {
                    ++i;
                }
                if (i < value.size()) {
                    ++i;
                }
            }
            continue;
        }

        result.push_back(static_cast<char>(c));
        ++i;
    }

    return result;
}

std::string CommandChecker::prepare_line(const std::string& line) const {
    std::string cleaned = strip_ansi_sequences(line);
    return case_sensitive_ ? cleaned : to_lower(cleaned);
}

void CommandChecker::log_expected_patterns() const {
    if (markers_.empty()) {
        logger().debug("No patterns defined for %s", name().c_str());
        return;
    }
    logger().debug("Expected patterns (%s match):",
                   match_mode_ == MatchMode::Any ? "any" : "all");
    for (const auto& marker : markers_) {
        logger().debug(" - %s", marker.c_str());
    }
}
