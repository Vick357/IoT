   README - Simulación de TurtleBot3 con LIDAR y SLAM en Docker

 Este documento detalla el paso a paso para implementar una simulación
 del robot TurtleBot3 con LIDAR, usando ROS Noetic y SLAM (gmapping) 
 dentro de un contenedor Docker. El objetivo es crear un entorno donde 
 el robot pueda mapear un entorno en tiempo real a partir de los datos del LIDAR.

   Paso 1: Crear el Dockerfile

 Se debe crear un archivo de texto llamado Dockerfile (sin extensión).
 Este archivo define la imagen personalizada de Docker que tendrá instalado:
 - ROS Noetic
 - Los paquetes del robot TurtleBot3
 - Los paquetes de simulación en Gazebo
 - El paquete SLAM gmapping

 Contenido del Dockerfile:

 FROM osrf/ros:noetic-desktop-full

 RUN apt-get update && apt-get install -y \
     ros-noetic-turtlebot3 \
     ros-noetic-turtlebot3-simulations \
     ros-noetic-slam-gmapping

 RUN echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc

 CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash && \
       roslaunch turtlebot3_gazebo turtlebot3_world.launch"]

   Paso 2: Construcción de la imagen Docker

 docker build -t turtlebot3-slam .

   Paso 3: Ejecutar el contenedor y lanzar Gazebo

 docker run -it --rm \
     --env="DISPLAY" \
     -v /tmp/.X11-unix:/tmp/.X11-unix \
     --network host \
     --name slam-bot \
     turtlebot3-slam

   Paso 4: Ejecutar el SLAM (gmapping)

 docker exec -it slam-bot bash -c "source /opt/ros/noetic/setup.bash && \
 roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping"

  Paso 5: Teleoperar el robot

 docker exec -it slam-bot bash -c "source /opt/ros/noetic/setup.bash && \
 rosrun turtlebot3_teleop turtlebot3_teleop_key"

   Paso 6: Visualizar el mapa en RViz
 docker exec -it slam-bot bash -c "source /opt/ros/noetic/setup.bash && \
 rosrun rviz rviz -d /opt/ros/noetic/share/turtlebot3_slam/rviz/turtlebot3_gmapping.rviz"

   Paso 7: Script para ejecutar múltiples entornos

 A continuación, se presenta un script que automatiza el lanzamiento de tres entornos dentro del contenedor Docker:
 El script ejecuta los siguientes comandos:

 1. Lanzar el mundo en Gazebo con el TurtleBot3
 2. Controlar el robot de forma teleoperada
 3. Visualizar el mapa en RViz con la configuración de SLAM (gmapping)

  Script: start_environments.sh
 #!/bin/bash
 
 # Entorno 1: Lanzar el mundo en Gazebo
 docker exec -it slam-bot bash -c "source /opt/ros/noetic/setup.bash && \
 roslaunch turtlebot3_gazebo turtlebot3_world.launch"
 
 # Entorno 2: Lanzar el control del robot
 docker exec -it slam-bot bash -c "source /opt/ros/noetic/setup.bash && \
 rosrun turtlebot3_teleop turtlebot3_teleop_key"

 # Entorno 3: Lanzar RViz con la configuración de SLAM
 docker exec -it slam-bot bash -c "source /opt/ros/noetic/setup.bash && \
 rosrun rviz rviz -d /opt/ros/noetic/share/turtlebot3_slam/rviz/turtlebot3_gmapping.rviz

 Este script debe ser ejecutado en la terminal como:

 ./start_environments.sh

  Trayectorias y mejoras realizadas

 Durante la simulación se probaron varias trayectorias:
 - Trayecto en forma de cuadrado
 - Trayecto en zigzag
 - Movimiento en espiral

 Mejoras sugeridas:
 - Automatizar el movimiento del robot
 - Ajustar la resolución y el rango del LIDAR
 - Agregar nodos de navegación
 - Montar la simulación en un entorno más complejo
