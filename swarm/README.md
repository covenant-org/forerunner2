## Start Gazebo with multiple drones

```bash
PX4_GZ_WORLD=imav_indoor PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="1,0,0,0,0,0" PX4_SIM_MODEL=gz_x500_lidar_down ./px4/build/px4_sitl_default/bin/px4 -i 0
```

> **Note:** You can find a [bash script](https://github.com/covenant-org/PX4-Autopilot/blob/main/spawn_swarm.sh) in the Covenant PX4 fork to automate launching multiple drones.

### Command components
- **PX4_GZ_WORLD**: Map name (URDF world)
- **PX4_SYS_AUTOSTART**: Drone model (e.g., 4001 for x500)
- **PX4_GZ_MODEL_POSE**: Initial drone position (set different values for each drone to avoid overlap)
- **PX4_SIM_MODEL**: Drone model name (matches URDF)
- **-i**: Drone instance number (increase by one for each drone)

## MAVLink communication

This launcher helps you scale and connect multiple drones using MAVLink.

```bash
./build/launch/launch --yaml-path swarm/multiple-dron-connection.yaml
```

## ISSUES:
- Port conflicts can occur when starting two MAVLink executables at the same time with launch.
    - A delay was tried as a workaround, but it did not solve the issue.