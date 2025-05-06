#!/bin/bash
cd /home/user/ros2_ws/src/warehouse_project/map_server/config
ros2 run nav2_map_server map_saver_cli -f warehouse_map_real
