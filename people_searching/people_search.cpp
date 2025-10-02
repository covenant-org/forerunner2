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
    std::cout<<"LLM Result received: isValidPerson=" << result.getIsValidPerson() << std::endl;
    if (result.getIsValidPerson()) {
        auto coords = result.getCoordinates();
        {
            std::lock_guard<std::mutex> lock(this->_person_mutex);
            this->_person_x = coords.getX();
            this->_person_y = coords.getY();
            this->_person_z = coords.getZ();
            this->_valid_person_found = true;
        }
        this->_logger.info("Valid person detected at (%.3f, %.3f, %.3f)",
                           coords.getX(), coords.getY(), coords.getZ());
    }
}

void PeopleSearch::get_position(const Core::IncomingMessage<Odometry>& msg) {
    auto odom = msg.content;
    auto pos = odom.getPosition();
    this->_drone_x = pos.getX();
    this->_drone_y = pos.getY();
    this->_drone_z = pos.getZ();
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
            // if (move_and_check(x_end, y_offset, z, 0.0f, wait)) return;
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
            // if (move_and_check(x_start, y_offset, z, 0.0f, wait)) return;
        } else if (movement_type == "v") {
            if (velocity_position_control(x_end, y_offset, z, 0.0f)) return;
            if (velocity_position_control(x_start, y_offset, z, 0.0f)) return;
        }
    }
}

bool PeopleSearch::velocity_position_control(float x, float y, float z, float yaw_rate, float max_vel) {
    float kp = 0.5f; // Proportional gain for position control
    float threshold = 0.4f; // Position error threshold to consider as "reached"
    // Limit maximum velocity // m/s
    float total_error = std::sqrt((x - _drone_x) * (x - _drone_x) +
                                  (y - _drone_y) * (y - _drone_y) +
                                  (z - _drone_z) * (z - _drone_z));

    float vx = 0.0f, vy = 0.0f, vz = 0.0f;
    float _vx_prev = 0.0f, _vy_prev = 0.0f, _vz_prev = 0.0f;

    // acceleration limits (m/s^2)
    float ax_max = 0.5f;
    float ay_max = 0.5f;
    float az_max = 0.5f;

    float alpha = 0.2f; // smoothing factor 0 < alpha < 1

    while (total_error > threshold) {
        float error_x = x - _drone_x;
        float error_y = y - _drone_y;
        float error_z = z - _drone_z;

        // float vx = kp * error_x;
        // float vy = kp * error_y;
        // float vz = kp * error_z;
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

        if (check_valid_person()) {
            send_velocity(0.0f, 0.0f, 0.0f, 0.0f); // Stop movement
            return true; // Exit if valid person found
        }
    }
    send_velocity(0.0f, 0.0f, 0.0f, 0.0f); // Stop movement
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

bool PeopleSearch::check_valid_person() {
    if (_valid_person_found.load()){
        std::lock_guard<std::mutex> lock(this->_person_mutex);
            // get current drone position
            std::cout << "Interrupting search! Flying to detected person at ("
                    << _person_x << ", " << _person_y << ", " << _person_z << ")" << std::endl;

            auto movement_type = this->get_argument<std::string>("--movement-type");
            if (movement_type == "v") {
                velocity_position_control(_person_x + _drone_x, _person_y + _drone_y, 4.0f, 0.0f, 5);
            }
            else if (movement_type == "p") {
                move_and_wait(_person_x + _drone_x, _person_y + _drone_y, 4.0f, 0.0f, 5);
            }

            land();
            return true; // Exit search after landing
    }
    return false;
}   

void PeopleSearch::run() {
    float x, y, z;

    // Retrieve the GPS target location argument
    auto gps_target = this->get_argument<std::vector<std::string>>("--gps-target-location");
    auto movement_type = this->get_argument<std::string>("--movement-type");
    if (gps_target.size() == 2) {
        std::cout << "GPS Target Location: Latitude = " << gps_target[0]
                  << ", Longitude = " << gps_target[1] << std::endl;
    } else {
        std::cerr << "Invalid GPS target location provided!" << std::endl;
        return;
    }

    x = std::stof(gps_target[0]);
    y = std::stof(gps_target[1]);
    z = -4.0f; // Fixed altitude of 4 meters
    
    // Takeoff to 4 meters
    std::cout << "Taking off to 4 meters..." << std::endl;
    auto request = this->_mission_client->new_msg();
    request.content.initTakeoff();
    request.content.getTakeoff().setDesiredAltitude(4.0);
    auto result = request.send();
    auto response = result.value().content;
    
    if (response.getCode() != 200) {
    this->_logger.error("Takeoff failed with code %d and message %s",
                        response.getCode(), response.getMessage());}

    // Move to initial position
    std::cout << "Moving to target location..." << std::endl;
    if (movement_type == "p") {
        move_and_wait(x, y, z, 0.0f, 15);
    } else if (movement_type == "v") {
        velocity_position_control(x, y, z, 0.0f, 5.0f);
    }
    sleep(2); // stabilize

    // Lawn-mower pattern search starting from center
    const float length = 24.0f;   // search box size (meters)
    const float step   = 4.0f;   // spacing between lines
    const int   wait   = 4;     // wait between moves (seconds)

    bool forward = true;

    std::cout << "Starting search pattern..." << std::endl;
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

    std::cout << "Search pattern completed." << std::endl;

    // Go home if no person found
    if (!check_valid_person()) {
        std::cout << "No valid person found. Returning to launch and landing..." << std::endl;
        velocity_position_control(0.0f, 0.0f, z, 0.0f); // Ascend to 4m
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