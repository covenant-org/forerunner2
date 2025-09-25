#include "argument_parser.hpp"
#include "people_search.hpp"
#include <chrono>
#include <thread>

PeopleSearch::PeopleSearch(Core::ArgumentParser parser) : Core::Vertex(parser) 
{
    this->_mission_client =
      this->create_action_client<MissionCommand, GenericResponse>("mission_command");
    this->_controller_client =
      this->create_action_client<Command, GenericResponse>("controller");
}

void PeopleSearch::move_and_wait(float x, float y, float z, float yaw_deg, int wait_sec) {
    this->send_coordinate(x, y, z, yaw_deg);
    std::this_thread::sleep_for(std::chrono::seconds(wait_sec));
}

void PeopleSearch::move_line_segmented(float x_start, float y, float length, float z, bool forward, float yaw, int wait) {
    float dir = forward ? 1.0f : -1.0f;

    // Segment lengths
    float step = length / 3.0f;

    // 1/3 point
    float x1 = x_start + dir * step;
    this->send_coordinate(x1, y, z, yaw);
    sleep(wait);

    // 2/3 point
    float x2 = x_start + dir * 2 * step;
    this->send_coordinate(x2, y, z, yaw);
    sleep(wait);

    // End point
    float x3 = x_start + dir * 3 * step;
    this->send_coordinate(x3, y, z, yaw);
    sleep(wait);
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

void PeopleSearch::run() {
    float north, east, down, yaw, x, y, z;

    bool valid_person_found = true;
    float person_x = 15.0f;
    float person_y = 11.0f;

    // Retrieve the GPS target location argument
    auto gps_target = this->get_argument<std::vector<std::string>>("--gps-target-location");
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
    std::cout << "1" << std::endl;
    auto result = request.send();
    std::cout << "2" << std::endl;
    auto response = result.value().content;
    std::cout << "3" << std::endl;
    
    if (response.getCode() != 200) {
    this->_logger.error("Takeoff failed with code %d and message %s",
                        response.getCode(), response.getMessage());}

    // Move to initial position
    std::cout << "Moving to target location..." << std::endl;
    this->send_coordinate(x, y, z, 0.0f);
    sleep(15); // Wait to reach position

    // Lawn-mower pattern search starting from center
    const float length = 20.0f;   // search box size (meters)
    const float step   = 4.0f;   // spacing between lines
    const int   wait   = 8;       // wait between moves (seconds)

    bool forward = true;

    std::cout << "Starting search pattern..." << std::endl;
    // Start at center line, expand outwards
    for (int i = 0; i <= length / (2 * step); i++) {
    // Current Y offset (positive and negative from center)
    float offset_up   = i * step;
    float offset_down = -i * step;

        // Skip the first "down" iteration since i=0 is the center line
        if (i == 0) {
            // Center line
            if (forward) {
                move_and_wait(x - length / 2, y + offset_up, z, 0.0f, wait);
                move_and_wait(x + length / 2, y + offset_up, z, 0.0f, wait);
            } else {
                move_and_wait(x + length / 2, y + offset_up, z, 0.0f, wait);
                move_and_wait(x - length / 2, y + offset_up, z, 0.0f, wait);
            }
            forward = !forward;
        } else {
            // Upper line
            if (forward) {
                move_and_wait(x - length / 2, y + offset_up, z, 0.0f, wait);
                move_and_wait(x + length / 2, y + offset_up, z, 0.0f, wait);
            } else {
                move_and_wait(x + length / 2, y + offset_up, z, 0.0f, wait);
                move_and_wait(x - length / 2, y + offset_up, z, 0.0f, wait);
            }
            forward = !forward;

            // Lower line
            if (forward) {
                move_and_wait(x - length / 2, y + offset_down, z, 0.0f, wait);
                move_and_wait(x + length / 2, y + offset_down, z, 0.0f, wait);
            } else {
                move_and_wait(x + length / 2, y + offset_down, z, 0.0f, wait);
                move_and_wait(x - length / 2, y + offset_down, z, 0.0f, wait);
            }
            forward = !forward;
        }
    }

        // Check for valid person detection (placeholder logic)
    if (valid_person_found) {
        std::cout << "Person detected at (" << person_x << ", " << person_y << "). Moving to investigate..." << std::endl;
        move_and_wait(person_x, person_y, z, 0.0f, 10);
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


    std::cout << "Search pattern completed." << std::endl;



}


int main(int argc, char **argv) {
    Core::BaseArgumentParser parser(argc, argv);

    parser.add_argument("--gps-target-location")
        .nargs(2)
        .help("GPS coordinates where the targes is likely to be (lat lon)");

    std::shared_ptr<PeopleSearch> people_search = std::make_shared<PeopleSearch>(parser);
    std::cout << "Starting People Search Node..." << std::endl;

    people_search->run();
    return 0;
}