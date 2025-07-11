# Forerunner

Structure:

incluir independecias (capnp, zmq, mavsdk) static?

- core/: Basic functionality for the pub/sub, action; similar to ros2 (cuidar limpieza) valgrind
    vertex
    pub/sub
    action
- messages/:
- mavlink/:
    telemetry
    action
        takeoff with height (2m)
        land
        keep_height
- camera
    zed
    realsense
    monocular nn
- planner
    algorithm
        - hockey
        - theta
    controller - comunicacion mavlink
- hooking
