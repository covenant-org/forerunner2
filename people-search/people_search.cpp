#include "argument_parser.hpp"
#include "people_search.hpp"
#include <chrono>
#include <cmath>
#include <thread>

PeopleSearch::PeopleSearch(Core::ArgumentParser parser) : Core::Vertex(parser) {
    this->_mission_client =
        this->create_action_client<MissionCommand, GenericResponse>("mission_command");
    this->_controller_client =
        this->create_action_client<Command, GenericResponse>("controller");
    this->_llm_subscriber = this->create_subscriber<LLMResult>(
        "llm_results", std::bind(&PeopleSearch::handle_llm_result, this, std::placeholders::_1));
    this->_odometry_subscriber = this->create_subscriber<Odometry>(
        "odometry", std::bind(&PeopleSearch::get_position, this, std::placeholders::_1));
}

void PeopleSearch::handle_llm_result(const Core::IncomingMessage<LLMResult>& msg) {
    auto result = msg.content;
    auto id = result.getObjectId();
    if (result.getIsValidPerson()) {
        auto coords = result.getCoordinates();
        auto now = std::chrono::steady_clock::now();
        float x_cam = coords.getX();
        float y_cam = coords.getY();
        float z_cam = coords.getZ();

        float yaw = _drone_yaw; // you need to store this from odometry
        float x_world = _drone_x + z_cam * cos(yaw) - x_cam * sin(yaw);
        float y_world = _drone_y + z_cam * sin(yaw) + x_cam * cos(yaw);
        float z_world = _drone_z; // keep altitude constant or adjust as needed

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

        this->_logger.info("Valid person detected at WORLD (%.3f, %.3f, %.3f) id=%u time=%lld",
                            x_world, y_world, z_world,
                           static_cast<unsigned>(id),
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
}

void PeopleSearch::move_and_wait(float x, float y, float z, float yaw_deg, int wait_sec) {
    this->send_coordinate(x, y, z, yaw_deg);
    std::this_thread::sleep_for(std::chrono::seconds(wait_sec));
}

bool PeopleSearch::move_and_check(float x, float y, float z, float yaw_deg, int wait_sec) {
    this->send_coordinate(x, y, z, yaw_deg);
    std::this_thread::sleep_for(std::chrono::seconds(wait_sec));
    return check_valid_person();
}

void PeopleSearch::fly_scan_line(float x_center, float y_offset, float z, float length, bool forward, int wait) {
    float x_start = x_center - length / 2;
    float x_end   = x_center + length / 2;

    int n_div = std::ceil(abs(x_end - x_start) / (4));

    auto movement_type = this->get_argument<std::string>("--movement-type");

    if (forward) {
        std::cout << "Moving x from " << x_start << " to " << x_end << " at y=" << y_offset << std::endl;
        
        if (movement_type == "p") {
            if (move_and_check(x_start, y_offset, z, 0.0f, wait)) return;
            for (int i = 0; i < n_div; i++) {
                float x_mid = x_start + (i + 1) * (length / n_div);
                if (move_and_check(x_mid, y_offset, z, 0.0f, wait)) return;
            }
        } else if (movement_type == "v") {
            if (velocity_position_control(x_start, y_offset, z, 0.0f)) return;
            if (velocity_position_control(x_end, y_offset, z, 0.0f)) return;
        }
    } else {
        std::cout << "Moving x from " << x_end << " to " << x_start << " at y=" << y_offset << std::endl;
        if (movement_type == "p") {
            if (move_and_check(x_end, y_offset, z, 0.0f, wait)) return;
            for (int i = 0; i < n_div; i++) {
                float x_mid = x_end - (i + 1) * (length / n_div);
                if (move_and_check(x_mid, y_offset, z, 0.0f, wait)) return;
            }
        } else if (movement_type == "v") {
            if (velocity_position_control(x_end, y_offset, z, 0.0f)) return;
            if (velocity_position_control(x_start, y_offset, z, 0.0f)) return;
        }
    }
}

bool PeopleSearch::velocity_position_control(float x, float y, float z, float yaw_rate, float max_vel, bool check_person) {
    
    float kp = 0.7f; // Proportional g  
    float threshold = 0.5f; // Position error threshold to consider as "reached"
    // Limit maximum velocity // m/s
    float total_error = std::sqrt((x - _drone_x) * (x - _drone_x) +
                                  (y - _drone_y) * (y - _drone_y) +
                                  (z - _drone_z) * (z - _drone_z));

    float vx = 0.0f, vy = 0.0f, vz = 0.0f;
    float _vx_prev = 0.0f, _vy_prev = 0.0f, _vz_prev = 0.0f;

    float alpha = 0.2f; // smoothing factor 0 < alpha < 1

    while (total_error > threshold) {
        std::cout << "Velocity control to (" << x << ", " << y << ", " << z << ") | Current pos: ("
                  << _drone_x << ", " << _drone_y << ", " << _drone_z << ") | Error: " << total_error << std::endl;
        float error_x = x - _drone_x;
        float error_y = y - _drone_y;
        float error_z = z - _drone_z;

        float vx_target = max_vel * std::tanh(kp * error_x / max_vel);
        float vy_target = max_vel * std::tanh(kp * error_y / max_vel);
        float vz_target = max_vel * std::tanh(kp * error_z / max_vel);

        vx = alpha * vx_target + (1 - alpha) * _vx_prev;
        vy = alpha * vy_target + (1 - alpha) * _vy_prev;
        vz = alpha * vz_target + (1 - alpha) * _vz_prev;

        _vx_prev = vx;
        _vy_prev = vy;
        _vz_prev = vz;

        float vel_mag = std::sqrt(vx * vx + vy * vy + vz * vz);
        if (vel_mag > max_vel) {
            vx = (vx / vel_mag) * max_vel;
            vy = (vy / vel_mag) * max_vel;
            vz = (vz / vel_mag) * max_vel;
        }

        send_velocity(vx, vy, vz, yaw_rate);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        total_error = std::sqrt((x - _drone_x) * (x - _drone_x) +
                                (y - _drone_y) * (y - _drone_y) +
                                (z - _drone_z) * (z - _drone_z));
        if (check_person)
            if (check_valid_person()) {
            std::cout << "Person found during velocity control, stopping." << std::endl;
            //send_velocity(0.0f, 0.0f, 0.0f, 0.0f); // Stop movement
            //send_coordinate(_drone_x, _drone_y, -4.0f, 0.0f);
            move_and_wait(_drone_x, _drone_y, -4.0f, 0.0f, 2);
            return true; // Exit if valid person found
            }
    }
    //send_velocity(0.0f, 0.0f, 0.0f, 0.0f); // Stop movement
    //send_coordinate(_drone_x, _drone_y, -4.0f, 0.0f);
    move_and_wait(_drone_x, _drone_y, -4.0f, 0.0f, 2);
    return false;

    }

void PeopleSearch::send_velocity(float vx, float vy, float vz, float yaw_rate) {
    auto request = _controller_client->new_msg();
    auto velocity = request.content.initVelocity();
    velocity.setX(vx);
    velocity.setY(vy);
    velocity.setZ(vz);
    velocity.setR(yaw_rate);

    auto result = request.send();
    if (!result.has_value()) {
        std::cerr << "Failed to send velocity: no response\n";
        return;
    }

    auto response = result.value().content;
    if (response.getCode() != 200) {
        std::cerr << "Velocity command rejected! Code: " << response.getCode()
                  << " Message: " << response.getMessage().cStr() << "\n";
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

bool PeopleSearch::confirm_valid_person(float person_x, float person_y, float person_z,
                                        std::chrono::steady_clock::time_point detection_time) {
    std::cout << "Confirming detected person at (" << person_x << ", " << person_y << ", " << person_z << ")" << std::endl;

    // Approach halfway (keep movement code outside of locks)
    auto movement_type = get_argument<std::string>("--movement-type");
    float approach_x = _drone_x + (person_x - _drone_x) / 4.0f;
    float approach_y = _drone_y + (person_y - _drone_y) / 4.0f;
    float person_distance = std::sqrt(person_x*person_x + person_y*person_y);
    if (person_distance > 2.0f)
    {
    if (movement_type == "v")
        velocity_position_control(approach_x, approach_y, -4.0f, 0.0f, 1.5, false);
        //std::cout << "What happens if we don't move to approach point?" << std::endl;
    else
        move_and_wait(approach_x, approach_y, -3.0f, 0.0f, 5);
    }

    // wait for a fresh detection (detection_time must be newer than snapshot_time)
    bool confirmed = false;
    const auto timeout = std::chrono::seconds(5);
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    std::unique_lock<std::mutex> lk(this->_person_mutex);
    // lambda predicate: new detection time is later than snapshot and spatially close
    auto predicate = [&](void)->bool {
        if (this->_last_detection_time <= detection_time) return false; // not new
        float dx = this->_person_x - person_x;
        float dy = this->_person_y - person_y;
        float dz = this->_person_z - person_z;
        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        const float threshold = 2.0f; // tune: 0.5..1.5 meters
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
        std::cout << "Person confirmed!" << std::endl;
        _last_confirmed_time = std::chrono::steady_clock::now();
        _last_confirmed_x = person_x;
        _last_confirmed_y = person_y;
        _last_confirmed_z = person_z;
        return true;
    }

    std::cout << "False positive or no fresh detection, returning to backup." << std::endl;
    // return to backup
    float drone_x_backup = _drone_x;
    float drone_y_backup = _drone_y;
    if (movement_type == "v") {
        velocity_position_control(drone_x_backup, drone_y_backup, -4.0f, 0.0f, 1.5, false);
    } else {
        move_and_wait(drone_x_backup, drone_y_backup, -4.0f, 0.0f, 1.5);
    }
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
        if (std::sqrt(dx*dx + dy*dy + dz*dz) < 2.0f)
            return false;
    }

    if (!_valid_person_found.load()) return false;

    //send_velocity(0.0f, 0.0f, 0.0f, 0.0f); // Stop movement
    send_coordinate(_drone_x, _drone_y, -4.0f, 0.0f);

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

    std::cout << "Starting person validation (snapshot at time="
              << detection_time.time_since_epoch().count() << ")" << std::endl;

    if (!confirm_valid_person(person_x, person_y, person_z, detection_time)) {
        _is_handling_person = false;
        std::cout << "Continue search" << std::endl;
        return false;
    }

    // proceed to go to person (use the locked snapshot or locking as needed)
    std::cout << "Interrupting search! Flying to detected person at ("
            << person_x << ", " << person_y << ", " << person_z << ")" << std::endl;

    auto movement_type = this->get_argument<std::string>("--movement-type");

    float person_distance = std::sqrt((person_x-_drone_x)*(person_x-_drone_x) +
                                      (person_y-_drone_y)*(person_y-_drone_y));

    if (person_distance< 2.0f) {
        std::cout << "Person very close, descending directly." << std::endl;
        //send_velocity(0.0f, 0.0f, 0.0f, 0.0f); 
        //send_coordinate(_drone_x, _drone_y, -4.0f, 0.0f);
        move_and_wait(_drone_x, _drone_y, -2.0f, 0.0f, 5);
        land();
        //send_coordinate(_drone_x, _drone_y, -0.0f, 0.0f);
        return true;
    }
    if (movement_type == "v") {
        std::cout << "Using velocity control to approach person." << std::endl;
        move_and_wait(person_x, person_y, -4.0f, 0.0f, 4);
    } else {
        move_and_wait(person_x + _drone_x, person_y + _drone_y, -4.0f, 0.0f, 5);
    }
    //send_velocity(0.0f, 0.0f, 0.0f, 0.0f); 
    //send_coordinate(_drone_x, _drone_y, -4.0f, 0.0f);
    move_and_wait(person_x, person_y, -2.0f, 0.0f, 5);
    land();
    //send_coordinate(_drone_x, _drone_y, -0.0f, 0.0f);
    return true;
}
 

void PeopleSearch::run() {
    float x, y, z;

    // Retrieve the GPS target location argument
    auto gps_target = this->get_argument<std::vector<std::string>>("--gps-target-location");
    auto movement_type = this->get_argument<std::string>("--movement-type");
    if (gps_target.size() == 2) {
        this->_logger.info("GPS target location provided: lat=%s, lon=%s",
                           gps_target[0].c_str(), gps_target[1].c_str());
    } else {
        this->_logger.error("GPS target location (--gps-target-location) is required.");
        return;
    }

    x = std::stof(gps_target[0]);
    y = std::stof(gps_target[1]);
    z = -4.0f; // Fixed altitude of 4 meters
    
    // Takeoff to 4 meters
    this->_logger.info("Initiating takeoff to 4.0 meters");
    auto request = this->_mission_client->new_msg();
    request.content.initTakeoff();
    request.content.getTakeoff().setDesiredAltitude(4.0);
    auto result = request.send();
    auto response = result.value().content;
    
    if (response.getCode() != 200) {
    this->_logger.error("Takeoff failed with code %d and message %s",
                        response.getCode(), response.getMessage());}

    auto setpoint_req = this->_controller_client->new_msg();
    auto wp = setpoint_req.content.initWaypoint();
    wp.setX(0);
    wp.setY(0);
    wp.setZ(z);
    auto setpoint_res = setpoint_req.send();
    auto setpoint_resp = setpoint_res.value().content;
    if (setpoint_resp.getCode() != 200) {
          this->_logger.error("Failed to set position setpoint before offboard: %s", 
                              setpoint_resp.getMessage().cStr());
          return;
        }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    //Offboard mode activation
    this->_logger.info("Activating offboard mode");

    auto cmd_req = this->_controller_client->new_msg();
    auto offboard_request = cmd_req.content.initOffboard();
    offboard_request.setEnable(true);
    auto cmd_result = cmd_req.send();
    auto cmd_response = cmd_result.value().content;
    if (cmd_response.getCode() != 200) {
        this->_logger.error("Offboard activation failed with code %d and message %s",
                            cmd_response.getCode(), cmd_response.getMessage());
    }

    // Move to initial position
    this->_logger.info("Moving to target GPS location (%.6f, %.6f) at %.2f meters",
                       x, y, -z);
    if (movement_type == "p") {
        move_and_wait(x, y, z, 0.0f, 15);
    } else if (movement_type == "v") {
        //velocity_position_control(x, y, z, 0.0f, 3.0f);
        // velocity command uses body frame 
        // TODO: use global velocity to get to gps-target
        move_and_wait(x, y, z, 0.0f, 15);
    }
    sleep(2); // stabilize

    // Lawn-mower pattern search starting from center
    const float length = 24.0f;   // search box size (meters)
    const float step   = 6.0f;   // spacing between lines
    const int   wait   = 4;     // wait between moves (seconds)

    bool forward = true;

    this->_logger.info("Starting lawn-mower search pattern...");
    // Start at center line, expand outwards
    for (int i = 0; i <= length / (2 * step); i++) {
        float offset_up   = y + i * step;
        float offset_down = y - i * step;

        fly_scan_line(x, offset_up, z, length, forward, wait);
        forward = !forward;

        if (i > 0) { // skip duplicate center
            fly_scan_line(x, offset_down, z, length, forward, wait);
            forward = !forward;
        }
    }

    this->_logger.info("Lawn-mower search pattern complete.");

    // Go home if no person found
    if (!check_valid_person()) {
        this->_logger.info("No valid person found, returning to home position.");
        move_and_wait(0.0f, 0.0f, -4.0f, 0.0f, 10);
        land();
    }

}

int main(int argc, char **argv) {
    Core::BaseArgumentParser parser(argc, argv);

    parser.add_argument("--gps-target-location")
        .nargs(2)
        .help("GPS coordinates where the targes is likely to be (lat lon)");

    parser.add_argument("--movement-type")
        .default_value("p")
        .help("Type of movement control: position (p) or velocity (v)");

    std::shared_ptr<PeopleSearch> people_search = std::make_shared<PeopleSearch>(parser);
    std::cout << "Starting People Search Node..." << std::endl;

    people_search->run();
    return 0;
}