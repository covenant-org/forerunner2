#include "argument_parser.hpp"
#include "people_search.hpp"

PeopleSearch::PeopleSearch(Core::ArgumentParser parser) : Core::Vertex(parser) 
{
    this->_mission_client =
      this->create_action_client<MissionCommand, GenericResponse>("mission_command");
    this->_controller_client =
      this->create_action_client<Command, GenericResponse>("controller");
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
    if (response.getCode() == 200) {
        std::cout << "Waypoint accepted by mavlink-mission\n";
    } else {
        std::cerr << "Waypoint rejected! Code: " << response.getCode()
                  << " Message: " << response.getMessage().cStr() << "\n";
    }
}

void PeopleSearch::run() {
    float north, east, down, yaw, x, y, z;

    std::vector<std::pair<float,float>> offsets = {
    {0, 0}, {10, 0}, {10, 10}, {0, 10}, {-10, 10}, {-10, 0}, {-10, -10}, {0, -10}
    };

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
    this->send_coordinate(x, y, z, 0.0f);

    // Search pattern
    for (auto& [dx, dy] : offsets) {
    this->send_coordinate(x + dx, y + dy, z, 0.0f);
    std::this_thread::sleep_for(std::chrono::seconds(8)); // wait between moves
    // TODO: integrate vision check here
    // if (vision_check()) { valid_person; break; }
    }

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