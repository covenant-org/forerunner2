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

## Setup

Clone repository:

```
git clone --recurse-submodules -j8 git@github.com:covenant-org/forerunner2.git
```

Compile third party dependencies:

```
cd vendor
cmake -Bbuild && cd build && cmake --build . && sudo cmake --install .
```

