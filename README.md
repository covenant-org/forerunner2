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

Compile third party dependencies (except MAVSDK & pcl):

```
cd vendor
cmake -Bbuild && cmake --build build && sudo cmake --install build
```

Compile and install MAVSDK

```
cd vendor
cmake -DCMAKE_BUILD_TYPE=Debug -Bbuildmav -SMAVSDK && sudo cmake --build buildmav --target install && sudo ldconfig
```

Compile and install PCL

```
cd vendor
cmake -DCMAKE_BUILD_TYPE=Debug -Bbuildpcl -Spcl && sudo cmake --build buildpcl && sudo cmake --install buildpcl
```

**Notes**:

- This is installed in a different step since there is a conflict with a target using the same name from `capnpproto`
- You can use `-GNinja` to build with Ninja instead of make (faster compilations)

Compile messages:

```
cd messages
cmake -Bbuild && cd build && cmake --build . && sudo cmake --install .
```

## Devcontainer

In order to ease the development and standardize the development enviroment we created a devcontainer.

### PX4

PX4 firmware is preinstalled and prebuild in the /usr/local/share/px4 folder. For easy access there is an alias _px4_ to make the targets. For example:

```
PX4_GZ_WORLD=imav_indoor px4 px4_sitl gz_x500_depth_mic
```
