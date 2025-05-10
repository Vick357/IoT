#!/bin/bash

# Entorno 1: Lanzar el mundo en Gazebo
docker exec -it slam-bot bash -c "source /opt/ros/noetic/setup.bash && roslaunch turtlebot3_gazebo turtlebot3_world.launch"

# Entorno 2: Lanzar el control del robot
docker exec -it slam-bot bash -c "source /opt/ros/noetic/setup.bash && rosrun turtlebot3_teleop turtlebot3_teleop_key"

# Entorno 3: Lanzar RViz con la configuración de SLAM
docker exec -it slam-bot bash -c "source /opt/ros/noetic/setup.bash && rosrun rviz rviz -d /opt/ros/noetic/share/turtlebot3_slam/rviz/turtlebot3_gmapping.rviz"

