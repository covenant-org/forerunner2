#include "people_search.hpp"

PeopleSearch::PeopleSearch(Core::ArgumentParser parser)
    : Core::Vertex(parser) 
{
    _mission_client = this->create_action_client<MissionCommand, GenericResponse>("mission_command");
}

void PeopleSearch::send_coordinate(float north, float east, float up, float yaw_deg) {
    auto request = _mission_client->new_msg();
    auto waypoint = request.content.initCoordinate();
    waypoint.setNorth(north);
    waypoint.setEast(east);
    waypoint.setUp(up);
    waypoint.setYaw(yaw_deg);

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
    while (true) {
        float north, east, up;
        std::cout << "Enter N E U coordinates (meters): ";
        std::cin >> north >> east >> up;
        send_coordinate(north, east, up);
    }
}
