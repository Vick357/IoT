FROM osrf/ros:noetic-desktop-full

# Instalar dependencias necesarias
RUN apt-get update && apt-get install -y \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-simulations \
    ros-noetic-slam-gmapping \
    x11-apps

# Configurar entorno
ENV TURTLEBOT3_MODEL=burger
RUN echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc

# Comando de entrada por defecto
CMD ["bash"]

