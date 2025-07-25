// By claude
#ifndef CORE_LOGGER_HPP
#define CORE_LOGGER_HPP

#include <chrono>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace Core {

enum class LogLevel { DEBUG = 0, INFO = 1, WARN = 2, ERROR = 3 };

class Logger {
 private:
  LogLevel _current_level;
  std::string _class_name;
  std::ofstream _log_file;

  std::string get_timestamp() const {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) %
              1000;

    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
  }

  std::string get_color(LogLevel level) const {
    switch (level) {
      case LogLevel::DEBUG:
        return "\033[90m";  // Light Gray
      case LogLevel::INFO:
        return "";  // Light Gray
      case LogLevel::WARN:
        return "\033[33m";  // Yellow
      case LogLevel::ERROR:
        return "\033[31m";  // Red
      default:
        return "\033[0m";  // Reset
    }
  }

  std::string get_level_string(LogLevel level) const {
    switch (level) {
      case LogLevel::DEBUG:
        return "DEBUG";
      case LogLevel::INFO:
        return "INFO ";
      case LogLevel::WARN:
        return "WARN ";
      case LogLevel::ERROR:
        return "ERROR";
      default:
        return "UNKN ";
    }
  }

  template <typename... Args>
  std::string format_string(const std::string& format, Args... args) {
    int size_s = std::snprintf(nullptr, 0, format.c_str(), args...) + 1;
    if (size_s <= 0) {
      throw std::runtime_error("Error during formatting.");
    }
    auto size = static_cast<size_t>(size_s);
    std::unique_ptr<char[]> buf(new char[size]);
    std::snprintf(buf.get(), size, format.c_str(), args...);
    return std::string(buf.get(), buf.get() + size - 1);
  }

  void log(LogLevel level, const std::string& message) {
    if (level < _current_level) return;

    std::string timestamp = get_timestamp();
    std::string level_str = get_level_string(level);
    std::string full_message = "[" + timestamp + "] [" + level_str + "]";

    if (!_class_name.empty()) {
      full_message += " [" + _class_name + "]";
    }

    full_message += " " + message;

    // Console output with colors
    std::cout << get_color(level) << full_message << "\033[0m" << std::endl;

    // File output without colors
    if (_log_file.is_open()) {
      _log_file << full_message << std::endl;
      _log_file.flush();
    }
  }

 public:
  Logger(LogLevel level = LogLevel::INFO,
         const std::string& filename = "application.log",
         const std::string& class_name = "")
      : _current_level(level),
        _class_name(class_name),
        _log_file(filename, std::ios::app) {
    if (!_log_file.is_open()) {
      std::cerr << "Warning: Could not open log file " << filename << std::endl;
    }
  }

  ~Logger() {
    if (_log_file.is_open()) {
      _log_file.close();
    }
  }

  void set_level(LogLevel level) { _current_level = level; }

  LogLevel get_level() const { return _current_level; }

  void set_classname(const std::string& name) { this->_class_name = name; }

  std::string get_classname() { return this->_class_name; }

  void debug(const std::string& message) { log(LogLevel::DEBUG, message); }

  template <typename... Args>
  void debug(const std::string& format, Args... args) {
    log(LogLevel::DEBUG, format_string(format, args...));
  }

  void info(const std::string& message) { log(LogLevel::INFO, message); }

  template <typename... Args>
  void info(const std::string& format, Args... args) {
    log(LogLevel::INFO, format_string(format, args...));
  }

  void warn(const std::string& message) { log(LogLevel::WARN, message); }

  template <typename... Args>
  void warn(const std::string& format, Args... args) {
    log(LogLevel::WARN, format_string(format, args...));
  }

  void error(const std::string& message) { log(LogLevel::ERROR, message); }

  template <typename... Args>
  void error(const std::string& format, Args... args) {
    log(LogLevel::ERROR, format_string(format, args...));
  }

  static LogLevel parse_level(const std::string& value) {
    if (value == "debug") {
      return LogLevel::DEBUG;
    } else if (value == "info") {
      return LogLevel::INFO;
    } else if (value == "warn") {
      return LogLevel::WARN;
    } else if (value == "error") {
      return LogLevel::ERROR;
    }

    throw std::runtime_error("invalid value for --log-level");
  }
};

}  // namespace Core

#endif
