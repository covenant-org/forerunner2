#ifndef PEOPLE_SEARCH_HPP
#define PEOPLE_SEARCH_HPP

#include "message.hpp"
#include "vertex.hpp"
#include <capnp_schemas/mavlink.capnp.h>
#include <capnp_schemas/controller.capnp.h>
#include <capnp_schemas/generics.capnp.h>
#include <capnp_schemas/detection_messages.capnp.h>
#include <memory>
#include <iostream>
#include <atomic>

class PeopleSearch : public Core::Vertex {
private:
    std::shared_ptr<Core::ActionClient<Command, GenericResponse>> _controller_client;
    std::shared_ptr<Core::ActionClient<MissionCommand, GenericResponse>> _mission_client;
    std::shared_ptr<Core::Subscriber<LLMResult>> _llm_subscriber;
    std::shared_ptr<Core::Subscriber<Odometry>> _odometry_subscriber;

    std::atomic<bool> _valid_person_found{false};
    std::mutex _person_mutex;
    float _person_x{0.0f}, _person_y{0.0f}, _person_z{0.0f};
    float _drone_x{0.0f}, _drone_y{0.0f}, _drone_z{0.0f};

public:
    PeopleSearch(Core::ArgumentParser);

    // Send a single coordinate (NED) to the drone
    void send_coordinate(float north, float east, float up, float yaw_deg = 0.0f);

    void move_and_wait(float x, float y, float z, float yaw_deg, int wait_sec);

    void move_line_segmented(float x_start, float y, float length, float z, bool forward, float yaw, int wait);

    void handle_llm_result(const Core::IncomingMessage<LLMResult>& msg);

    bool check_valid_person();

    void get_position(const Core::IncomingMessage<Odometry>& msg);

    // Main loop: read coordinates from user input
    void run() override;
};

#endif // PEOPLE_SEARCH_HPP
