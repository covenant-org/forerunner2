PX4_GZ_WORLD=imav_indoor PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="1,0,0,0,0,0" PX4_SIM_MODEL=gz_x500_lidar_down px4_sitl_bin -i 0

PX4_GZ_WORLD=imav_indoor PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="2,0,0,0,0,0" PX4_SIM_MODEL=gz_x500 px4_sitl_bin -i 1

# Ejecutar codigos a los que es dependiente mi proyecto
./build/launch/launch --yaml-path swarm/multiple-dron-connection.yaml