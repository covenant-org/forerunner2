#pragma once

#include "base_checker.hpp"

#include <string>
#include <vector>

class CommandChecker : public BaseChecker {
public:
    enum class MatchMode { Any, All };

    CommandChecker(Core::Logger& logger,
                   std::string name,
                   std::string command,
                   std::vector<std::string> expected_markers,
                   std::string command_error_message,
                   std::string success_message,
                   std::string failure_message,
                   MatchMode match_mode = MatchMode::Any,
                   bool case_sensitive = false);

    DiagnosticResult run() override;

private:
    bool evaluate_output(const std::string& output,
                         std::vector<std::string>& matched_lines,
                         std::vector<std::string>& missing_markers) const;

    std::string to_lower(const std::string& value) const;
    void log_expected_patterns() const;

    std::string command_;
    std::vector<std::string> markers_;
    std::vector<std::string> markers_processed_;
    std::string command_error_message_;
    std::string success_message_;
    std::string failure_message_;
    MatchMode match_mode_;
    bool case_sensitive_;
};
