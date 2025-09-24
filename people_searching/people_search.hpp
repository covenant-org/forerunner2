#ifndef PEOPLE_SEARCH_HPP
#define PEOPLE_SEARCH_HPP

#include "message.hpp"
#include "vertex.hpp"
#include <capnp_schemas/mavlink.capnp.h>
#include <capnp_schemas/controller.capnp.h>
#include <capnp_schemas/generics.capnp.h>
#include <memory>
#include <iostream>

class PeopleSearch : public Core::Vertex {
private:
    std::shared_ptr<Core::ActionClient<Command, GenericResponse>> _controller_client;
    std::shared_ptr<Core::ActionClient<MissionCommand, GenericResponse>> _mission_client;

public:
    PeopleSearch(Core::ArgumentParser);

    // Send a single coordinate (NED) to the drone
    void send_coordinate(float north, float east, float up, float yaw_deg = 0.0f);

    void move_and_wait(float x, float y, float z, float yaw_deg, int wait_sec);

    void move_line_segmented(float x_start, float y, float length, float z, bool forward, float yaw, int wait);

    // Main loop: read coordinates from user input
    void run() override;
};

#endif // PEOPLE_SEARCH_HPP
