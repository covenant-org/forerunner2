#include "argument_parser.hpp"
#include "people_search.hpp"
#include <bits/stdc++.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <tuple>

PeopleSearch::PeopleSearch(Core::ArgumentParser parser) : Core::Vertex(parser) {
  this->_mission_client =
      this->create_action_client<MissionCommand, GenericResponse>(
          "mission_command");
  this->_controller_client =
      this->create_action_client<Command, GenericResponse>("controller");
  this->_llm_subscriber = this->create_subscriber<LLMResult>(
      "llm_results",
      std::bind(&PeopleSearch::handle_llm_result, this, std::placeholders::_1));
  this->_odometry_subscriber = this->create_subscriber<Odometry>(
      "odometry",
      std::bind(&PeopleSearch::get_position, this, std::placeholders::_1));
  this->_path_publisher = this->create_publisher<Point>("path_point");
}

void PeopleSearch::handle_llm_result(
    const Core::IncomingMessage<LLMResult>& msg) {
  auto result = msg.content;
  auto id = result.getObjectId();
  if (result.getIsValidPerson()) {
    auto coords = result.getCoordinates();
    auto now = std::chrono::steady_clock::now();
    float x_cam = coords.getX();
    float y_cam = coords.getY();
    float z_cam = coords.getZ();

    float yaw = _drone_yaw;  // you need to store this from odometry
    float x_world = _drone_x + x_cam * cos(yaw) - y_cam * sin(yaw);
    float y_world = _drone_y + x_cam * sin(yaw) + y_cam * cos(yaw);
    float z_world = _drone_z;  // keep altitude constant or adjust as needed

    {
      std::lock_guard<std::mutex> lock(this->_person_mutex);
      // remove test offset or make it configurable
      this->_person_x = x_world;
      this->_person_y = y_world;
      this->_person_z = z_world;
      this->_last_detection_time = now;
      this->_valid_person_found.store(true);
    }
    // notify waiting confirmers
    this->_person_cv.notify_one();

    this->_logger.info(
        "Valid person detected at WORLD (%.3f, %.3f, %.3f) id=%u time=%lld",
        x_world, y_world, z_world, static_cast<unsigned>(id),
        static_cast<long long>(now.time_since_epoch().count()));
  } else {
    this->_logger.info("Detection ID %u: NOT a valid person", id);
    {
      std::lock_guard<std::mutex> lock(this->_person_mutex);
      this->_valid_person_found.store(false);
      // optionally clear timestamp
      // _last_detection_time = std::chrono::steady_clock::time_point::min();
    }
  }
}

void PeopleSearch::get_position(const Core::IncomingMessage<Odometry>& msg) {
  auto odom = msg.content;
  auto pos = odom.getPosition();
  this->_drone_x = pos.getX();
  this->_drone_y = pos.getY();
  this->_drone_z = pos.getZ();

  auto q = odom.getQ();
  // Convert quaternion to yaw (in radians)
  float siny_cosp = 2 * (q.getW() * q.getZ() + q.getX() * q.getY());
  float cosy_cosp = 1 - 2 * (q.getY() * q.getY() + q.getZ() * q.getZ());
  float yaw = std::atan2(siny_cosp, cosy_cosp);
  this->_drone_yaw = yaw;

  if (!this->_home_set.load(std::memory_order_acquire)) {
    this->_home_x = this->_drone_x;
    this->_home_y = this->_drone_y;
    this->_home_z = this->_drone_z;
    this->_home_set.store(true, std::memory_order_release);
  }

  this->_has_odometry.store(true, std::memory_order_release);
}

void PeopleSearch::move_and_wait(float x, float y, float z, float yaw_deg) {
  this->send_coordinate(x, y, z, yaw_deg);

  const float position_threshold = 1.0f;  // 50cm tolerance

  while (true) {
    // Calculate distance to target
    float dx = _drone_x - x;
    float dy = _drone_y - y;
    float dz = _drone_z - z;
    float distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Check if reached target
    if (distance < position_threshold) {
      this->_logger.debug(
          "Reached waypoint (%.2f, %.2f, %.2f), distance: %.3fm", x, y, z,
          distance);
      break;
    }

    // Sleep briefly before next check
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

bool PeopleSearch::move_and_check(float x, float y, float z, float yaw_deg) {
  move_and_wait(x, y, z, yaw_deg);
  return check_valid_person();
}

void PeopleSearch::fly_scan_line(float x_center, float y_offset, float z,
                                 float length, bool forward) {
  float x_start = x_center - length / 2;
  float x_end = x_center + length / 2;

  int n_div = std::ceil(abs(x_end - x_start) / (4));

  if (forward) {
    this->_logger.debug("Moving x from %.2f to %.2f at y=%.2f", x_start, x_end,
                        y_offset);

    if (move_and_check(x_start, y_offset, z, 0.0f)) return;
    for (int i = 0; i < n_div; i++) {
      float x_mid = x_start + (i + 1) * (length / n_div);
      auto msg = this->_path_publisher->new_msg();
      msg.content.setX(static_cast<float>(x_mid));
      msg.content.setY(static_cast<float>(y_offset));
      msg.content.setZ(static_cast<float>(z));
      msg.publish();
      if (move_and_check(x_mid, y_offset, z, 0.0f)) return;
    }
  } else {
    this->_logger.debug("Moving x from %.2f to %.2f at y=%.2f", x_end, x_start,
                        y_offset);
    if (move_and_check(x_end, y_offset, z, 0.0f)) return;
    for (int i = 0; i < n_div; i++) {
      float x_mid = x_end - (i + 1) * (length / n_div);
      auto msg = this->_path_publisher->new_msg();
      msg.content.setX(static_cast<float>(x_mid));
      msg.content.setY(static_cast<float>(y_offset));
      msg.content.setZ(static_cast<float>(z));
      msg.publish();
      if (move_and_check(x_mid, y_offset, z, 0.0f)) return;
    }
  }
}

void PeopleSearch::calculateWaypoints(
    float x_center, float y_offset, float z, float length, bool forward,
    std::list<std::tuple<float, float, float>>& waypoints) {
  float x_start = x_center - length / 2;
  float x_end = x_center + length / 2;

  int n_div = std::ceil(abs(x_end - x_start) / (4));

  if (forward) {
    this->_logger.debug("Moving x from %.2f to %.2f at y=%.2f", x_start, x_end,
                        y_offset);

    waypoints.push_back(std::make_tuple(x_start, y_offset, z));
    for (int i = 0; i < n_div; i++) {
      float x_mid = x_start + (i + 1) * (length / n_div);
      waypoints.push_back(std::make_tuple(x_mid, y_offset, z));
    }
  } else {
    this->_logger.debug("Moving x from %.2f to %.2f at y=%.2f", x_end, x_start,
                        y_offset);
    waypoints.push_back(std::make_tuple(x_end, y_offset, z));
    for (int i = 0; i < n_div; i++) {
      float x_mid = x_end - (i + 1) * (length / n_div);
      waypoints.push_back(std::make_tuple(x_mid, y_offset, z));
    }
  }
}

void PeopleSearch::land() {
  this->_logger.info("Initiating landing sequence.");
  auto request = this->_mission_client->new_msg();
  request.content.setLand();
  request.content.getLand();
  auto result = request.send();
  auto response = result.value().content;
  if (response.getCode() != 200) {
    this->_logger.error("Landing failed with code %d and message %s",
                        response.getCode(), response.getMessage());
  }
}

void PeopleSearch::send_coordinate(float x, float y, float z, float yaw_deg) {
  auto request = _controller_client->new_msg();
  auto waypoint = request.content.initWaypoint();
  waypoint.setX(static_cast<float>(x));
  waypoint.setY(static_cast<float>(y));
  waypoint.setZ(static_cast<float>(z));
  waypoint.setR(static_cast<float>(yaw_deg));

  auto result = request.send();
  if (!result.has_value()) {
    std::cerr << "Failed to send coordinate: no response\n";
    return;
  }

  auto response = result.value().content;
  if (response.getCode() != 200) {
    std::cerr << "Waypoint rejected! Code: " << response.getCode()
              << " Message: " << response.getMessage().cStr() << "\n";
  }
}

bool PeopleSearch::confirm_valid_person(
    float person_x, float person_y, float person_z,
    std::chrono::steady_clock::time_point detection_time) {
  this->_logger.info(
      "Confirming person at (%.3f, %.3f, %.3f) detected at time=%lld", person_x,
      person_y, person_z,
      static_cast<long long>(detection_time.time_since_epoch().count()));

  // Approach halfway (keep movement code outside of locks)
  float approach_x = _drone_x + (person_x - _drone_x) / 4.0f;
  float approach_y = _drone_y + (person_y - _drone_y) / 4.0f;
  float person_distance = std::sqrt(person_x * person_x + person_y * person_y);
  if (person_distance > 2.0f) {
    move_and_wait(approach_x, approach_y, -3.0f, 0.0f);
  }
  // wait for a fresh detection (detection_time must be newer than
  // snapshot_time)
  bool confirmed = false;
  const auto timeout = std::chrono::seconds(5);
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  std::unique_lock<std::mutex> lk(this->_person_mutex);
  // lambda predicate: new detection time is later than snapshot and spatially
  // close
  auto predicate = [&](void) -> bool {
    if (this->_last_detection_time <= detection_time) return false;  // not new
    float dx = this->_person_x - person_x;
    float dy = this->_person_y - person_y;
    float dz = this->_person_z - person_z;
    float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    const float threshold = 2.0f;  // tune: 0.5..1.5 meters
    return dist <= threshold;
  };

  // wait until predicate or timeout
  if (_person_cv.wait_until(lk, deadline, predicate)) {
    // predicate true: fresh detection within spatial threshold
    confirmed = true;
    // consume detection
    this->_valid_person_found.store(false);
  }
  lk.unlock();

  if (confirmed) {
    this->_logger.info("Person confirmed at (%.3f, %.3f, %.3f)", person_x,
                       person_y, person_z);
    _last_confirmed_time = std::chrono::steady_clock::now();
    _last_confirmed_x = person_x;
    _last_confirmed_y = person_y;
    _last_confirmed_z = person_z;
    return true;
  }

  this->_logger.info("Person NOT confirmed, resuming search");
  // return to backup
  float drone_x_backup = _drone_x;
  float drone_y_backup = _drone_y;

  move_and_wait(drone_x_backup, drone_y_backup, -3.0f, 0.0f);
  return false;
}

bool PeopleSearch::check_valid_person() {
  auto now = std::chrono::steady_clock::now();

  // Skip if we are already handling a confirmed person
  if (_is_handling_person) return false;

  // Cooldown: ignore detections within 5s and 2m of last confirmed
  const auto cooldown = std::chrono::seconds(5);
  if (now - _last_confirmed_time < cooldown) {
    float dx = _person_x - _last_confirmed_x;
    float dy = _person_y - _last_confirmed_y;
    float dz = _person_z - _last_confirmed_z;
    if (std::sqrt(dx * dx + dy * dy + dz * dz) < 2.0f) return false;
  }

  if (!_valid_person_found.load()) return false;

  send_coordinate(_drone_x, _drone_y, -3.0f, 0.0f);

  float person_x, person_y, person_z;
  std::chrono::steady_clock::time_point detection_time;
  {
    std::lock_guard<std::mutex> lock(this->_person_mutex);
    person_x = _person_x;
    person_y = _person_y;
    person_z = _person_z;
    detection_time = _last_detection_time;
    // do NOT clear _valid_person_found here — let confirm consume/clear it
  }

  _is_handling_person = true;

  this->_logger.info(
      "Starting person validation at (%.3f, %.3f, %.3f) detected at time=%lld",
      person_x, person_y, person_z,
      static_cast<long long>(detection_time.time_since_epoch().count()));

  if (!confirm_valid_person(person_x, person_y, person_z, detection_time)) {
    _is_handling_person = false;
    this->_logger.info("Person validation failed, resuming search");
    return false;
  }

  // proceed to go to person (use the locked snapshot or locking as needed)
  this->_logger.info("Navigating to confirmed person at (%.3f, %.3f, %.3f)",
                     person_x, person_y, person_z);

  float person_distance =
      std::sqrt((person_x - _drone_x) * (person_x - _drone_x) +
                (person_y - _drone_y) * (person_y - _drone_y));

  if (person_distance < 2.0f) {
    this->_logger.info(
        "Person is within %.2f meters, initiating landing sequence",
        person_distance);
    // send_coordinate(_drone_x, _drone_y, -4.0f, 0.0f);
    land();
    // send_coordinate(_drone_x, _drone_y, -0.0f, 0.0f);
    return true;
  }
  // send_coordinate(_drone_x, _drone_y, -4.0f, 0.0f);
  move_and_wait(person_x, person_y, -3.0f, 0.0f);
  land();
  // send_coordinate(_drone_x, _drone_y, -0.0f, 0.0f);
  return true;
}

void PeopleSearch::run() {
  const float offset_x = this->get_argument<float>("--x");
  const float offset_y = this->get_argument<float>("--y");
  float takeoff_altitude = std::fabs(this->get_argument<float>("--altitude"));
  if (takeoff_altitude <= 0.0f) {
    this->_logger.warn(
        "Configured altitude %.2f is non-positive, defaulting to 3.0m",
        takeoff_altitude);
    takeoff_altitude = 3.0f;
  }
  const float search_altitude = -takeoff_altitude;

  if (!this->_has_odometry.load(std::memory_order_acquire)) {
    this->_logger.info("Waiting for initial odometry...");
    while (!this->_has_odometry.load(std::memory_order_acquire)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  float start_x = this->_drone_x;
  float start_y = this->_drone_y;

  const float center_x = start_x + offset_x;
  const float center_y = start_y + offset_y;

  // Takeoff to 4 meters
  this->_logger.info("Initiating takeoff to %.2f meters", takeoff_altitude);
  auto request = this->_mission_client->new_msg();
  request.content.initTakeoff();
  request.content.getTakeoff().setDesiredAltitude(takeoff_altitude);
  auto result = request.send();
  auto response = result.value().content;

  if (response.getCode() != 200) {
    this->_logger.error("Takeoff failed with code %d and message %s",
                        response.getCode(), response.getMessage());
  }

  auto setpoint_req = this->_controller_client->new_msg();
  auto wp = setpoint_req.content.initWaypoint();
  wp.setX(this->_drone_x);
  wp.setY(this->_drone_y);
  wp.setZ(search_altitude);
  auto setpoint_res = setpoint_req.send();
  auto setpoint_resp = setpoint_res.value().content;
  if (setpoint_resp.getCode() != 200) {
    this->_logger.error("Failed to set position setpoint before offboard: %s",
                        setpoint_resp.getMessage().cStr());
    return;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Offboard mode activation
  this->_logger.info("Activating offboard mode");

  auto cmd_req = this->_controller_client->new_msg();
  auto offboard_request = cmd_req.content.initOffboard();
  offboard_request.setEnable(true);
  auto cmd_result = cmd_req.send();
  auto cmd_response = cmd_result.value().content;
  if (cmd_response.getCode() != 200) {
    this->_logger.error(
        "Offboard activation failed with code %d and message %s",
        cmd_response.getCode(), cmd_response.getMessage());
  }

  // Move to initial position
  this->_logger.info(
      "Moving to target GPS location (%.6f, %.6f) at %.2f meters", center_x,
      center_y, takeoff_altitude);
  move_and_wait(center_x, center_y, search_altitude, 0.0f);

  // Lawn-mower pattern search starting from center
  const float length = 10.0f;  // search box size (meters)
  const float step = 2.0f;     // spacing between lines

  bool forward = true;

  this->_logger.info("Starting lawn-mower search pattern...");

  // calculate waypoints
  std::list<std::tuple<float, float, float>> waypoints;

  for (int i = 0; i <= length / (2 * step); i++) {
    float offset_up = center_y + i * step;
    float offset_down = center_y - i * step;

    calculateWaypoints(center_x, offset_up, search_altitude, length, forward,
                       waypoints);
    forward = !forward;

    if (i > 0) {  // skip duplicate center
      calculateWaypoints(center_x, offset_down, search_altitude, length,
                         forward, waypoints);
      forward = !forward;
    }
  }

  for (int i = 0; i <= length / (2 * step); i++) {
    float offset_up = center_y + i * step;
    float offset_down = center_y - i * step;

    if (this->_valid_person_found) {
      break;
    }

    fly_scan_line(center_x, offset_up, search_altitude, length, forward);
    forward = !forward;

    if (i > 0) {  // skip duplicate center
      fly_scan_line(center_x, offset_down, search_altitude, length, forward);
      forward = !forward;
    }
  }

  this->_logger.info("Lawn-mower search pattern complete.");

  // Go home if no person found
  if (!check_valid_person()) {
    this->_logger.info("No valid person found, returning to home position.");
    float home_x = this->_home_set.load(std::memory_order_acquire)
                       ? this->_home_x
                       : this->_drone_x;
    float home_y = this->_home_set.load(std::memory_order_acquire)
                       ? this->_home_y
                       : this->_drone_y;
    move_and_wait(home_x, home_y, search_altitude, 0.0f);
    land();
  }
}

int main(int argc, char** argv) {
  Core::BaseArgumentParser parser(argc, argv);

  parser.add_argument("--x").scan<'g', float>().help("x distance in meters");
  parser.add_argument("--y").scan<'g', float>().help("x distance in meters");
  parser.add_argument("--altitude")
      .scan<'g', float>()
      .default_value(3.0f)
      .help("Takeoff altitude in meters (positive, default 3.0)");

  std::shared_ptr<PeopleSearch> people_search =
      std::make_shared<PeopleSearch>(parser);

  people_search->run();
  return 0;
}
