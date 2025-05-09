# Robot SLAM Simulation using ROS and Docker

## Descripción del Proyecto

Este proyecto implementa un entorno simulado para un robot equipado con sensor LIDAR que realiza SLAM (Simultaneous Localization and Mapping). Utilizando ROS (Robot Operating System) y Docker, este entorno permite:

- Simular un robot simple con un sensor LIDAR en un entorno virtual
- Implementar algoritmos de SLAM para generar mapas del entorno
- Controlar el robot mediante teclado
- Visualizar en tiempo real el robot, los datos del sensor y el mapa generado

Todo el sistema está empaquetado en un contenedor Docker para garantizar una ejecución consistente en cualquier entorno compatible.

## Contenido del Proyecto

### Archivos Principales

- `Dockerfile`: Define el entorno completo de simulación:

```Dockerfile

# Dockerfile para un robot simple con SLAM usando Stage con interfaz gráfica
FROM ros:noetic

# Instalar dependencias
RUN apt-get update && apt-get install -y \
    git \
    ros-noetic-gmapping \
    ros-noetic-map-server \
    ros-noetic-tf \
    ros-noetic-rviz \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-navigation \
    ros-noetic-xacro \
    ros-noetic-urdf \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-robot-state-publisher \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Crear espacio de trabajo
WORKDIR /catkin_ws/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    catkin_init_workspace"

# Crear el paquete personalizado
RUN mkdir -p simple_lidar_example/launch
RUN mkdir -p simple_lidar_example/urdf
RUN mkdir -p simple_lidar_example/rviz

# Crear archivo package.xml
RUN echo '<?xml version="1.0"?> \
\n<package format="2"> \
\n  <name>simple_lidar_example</name> \
\n  <version>0.0.1</version> \
\n  <description>A simple LIDAR SLAM example</description> \
\n  <maintainer email="user@example.com">User</maintainer> \
\n  <license>BSD</license> \
\n  <buildtool_depend>catkin</buildtool_depend> \
\n  <depend>roscpp</depend> \
\n  <depend>gmapping</depend> \
\n  <depend>map_server</depend> \
\n  <depend>tf</depend> \
\n  <depend>sensor_msgs</depend> \
\n  <depend>geometry_msgs</depend> \
\n  <depend>rviz</depend> \
\n  <depend>urdf</depend> \
\n  <depend>xacro</depend> \
\n  <depend>robot_state_publisher</depend> \
\n</package>' > /catkin_ws/src/simple_lidar_example/package.xml

# Crear CMakeLists.txt
RUN echo 'cmake_minimum_required(VERSION 3.0.2) \
\nproject(simple_lidar_example) \
\nfind_package(catkin REQUIRED COMPONENTS \
\n  roscpp \
\n  tf \
\n  sensor_msgs \
\n  geometry_msgs \
\n) \
\ncatkin_package() \
\ninclude_directories(${catkin_INCLUDE_DIRS}) \
\ninstall(DIRECTORY launch urdf rviz \
\n  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} \
\n)' > /catkin_ws/src/simple_lidar_example/CMakeLists.txt

# Crear un nodo simple que simule datos de LIDAR y movimiento del robot
RUN echo '#include <ros/ros.h> \
\n#include <sensor_msgs/LaserScan.h> \
\n#include <tf/transform_broadcaster.h> \
\n#include <geometry_msgs/Twist.h> \
\n \
\nclass RobotSimulator { \
\nprivate: \
\n  ros::NodeHandle nh_; \
\n  ros::Publisher scan_pub_; \
\n  ros::Subscriber cmd_vel_sub_; \
\n  tf::TransformBroadcaster br_; \
\n  ros::Timer timer_; \
\n \
\n  // Robot state \
\n  double x_ = 0.0; \
\n  double y_ = 0.0; \
\n  double theta_ = 0.0; \
\n \
\n  // Robot commands \
\n  double linear_vel_ = 0.0; \
\n  double angular_vel_ = 0.0; \
\n \
\npublic: \
\n  RobotSimulator() { \
\n    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 50); \
\n    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10, &RobotSimulator::cmdVelCallback, this); \
\n \
\n    // Set up timer for updates (10Hz) \
\n    timer_ = nh_.createTimer(ros::Duration(0.1), &RobotSimulator::update, this); \
\n  } \
\n \
\n  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) { \
\n    linear_vel_ = msg->linear.x; \
\n    angular_vel_ = msg->angular.z; \
\n  } \
\n \
\n  void update(const ros::TimerEvent&) { \
\n    // Update robot position based on velocity commands \
\n    ros::Time current_time = ros::Time::now(); \
\n    double dt = 0.1; // 10Hz update rate \
\n \
\n    // Update position \
\n    theta_ += angular_vel_ * dt; \
\n    x_ += linear_vel_ * cos(theta_) * dt; \
\n    y_ += linear_vel_ * sin(theta_) * dt; \
\n \
\n    // Broadcast the odom->base_link transform \
\n    tf::Transform odom_transform; \
\n    odom_transform.setOrigin(tf::Vector3(x_, y_, 0.0)); \
\n    tf::Quaternion q; \
\n    q.setRPY(0, 0, theta_); \
\n    odom_transform.setRotation(q); \
\n    br_.sendTransform(tf::StampedTransform(odom_transform, current_time, "odom", "base_link")); \
\n \
\n    // Broadcast the base_link->base_laser transform \
\n    tf::Transform laser_transform; \
\n    laser_transform.setOrigin(tf::Vector3(0.1, 0.0, 0.0)); \
\n    laser_transform.setRotation(tf::Quaternion(0, 0, 0, 1)); \
\n    br_.sendTransform(tf::StampedTransform(laser_transform, current_time, "base_link", "base_laser")); \
\n \
\n    // Create and publish laser scan data \
\n    publishLaserScan(current_time); \
\n  } \
\n \
\n  void publishLaserScan(const ros::Time& current_time) { \
\n    sensor_msgs::LaserScan scan; \
\n    scan.header.stamp = current_time; \
\n    scan.header.frame_id = "base_laser"; \
\n    scan.angle_min = -2.0; \
\n    scan.angle_max = 2.0; \
\n    scan.angle_increment = 0.01; \
\n    scan.time_increment = 0.0; \
\n    scan.scan_time = 0.1; \
\n    scan.range_min = 0.1; \
\n    scan.range_max = 10.0; \
\n \
\n    // Calculate number of readings \
\n    int num_readings = (scan.angle_max - scan.angle_min) / scan.angle_increment; \
\n    scan.ranges.resize(num_readings); \
\n \
\n    // Simulating a square room (4 walls) - size 6x6 meters \
\n    float room_size = 6.0; \
\n    float half_size = room_size / 2.0; \
\n \
\n    for (int i = 0; i < num_readings; i++) { \
\n      float angle = scan.angle_min + i * scan.angle_increment; \
\n      float global_angle = angle + theta_; \
\n \
\n      // Check distance to each wall from current robot position \
\n      float distances[4]; // distances to front, right, back, left walls \
\n \
\n      // Front wall (y = half_size) \
\n      if (sin(global_angle) > 0.001) { \
\n        distances[0] = (half_size - y_) / sin(global_angle); \
\n      } else { \
\n        distances[0] = 999.0; \
\n      } \
\n \
\n      // Right wall (x = half_size) \
\n      if (cos(global_angle) > 0.001) { \
\n        distances[1] = (half_size - x_) / cos(global_angle); \
\n      } else { \
\n        distances[1] = 999.0; \
\n      } \
\n \
\n      // Back wall (y = -half_size) \
\n      if (sin(global_angle) < -0.001) { \
\n        distances[2] = (-half_size - y_) / sin(global_angle); \
\n      } else { \
\n        distances[2] = 999.0; \
\n      } \
\n \
\n      // Left wall (x = -half_size) \
\n      if (cos(global_angle) < -0.001) { \
\n        distances[3] = (-half_size - x_) / cos(global_angle); \
\n      } else { \
\n        distances[3] = 999.0; \
\n      } \
\n \
\n      // Find minimum positive distance \
\n      float min_dist = scan.range_max; \
\n      for (int j = 0; j < 4; j++) { \
\n        if (distances[j] > 0 && distances[j] < min_dist) { \
\n          min_dist = distances[j]; \
\n        } \
\n      } \
\n \
\n      // Set range value \
\n      scan.ranges[i] = min_dist; \
\n      if (min_dist > scan.range_max) { \
\n        scan.ranges[i] = scan.range_max; \
\n      } \
\n    } \
\n \
\n    scan_pub_.publish(scan); \
\n  } \
\n}; \
\n \
\nint main(int argc, char** argv) { \
\n  ros::init(argc, argv, "robot_simulator"); \
\n \
\n  RobotSimulator robot_simulator; \
\n \
\n  ros::spin(); \
\n \
\n  return 0; \
\n}' > /catkin_ws/src/simple_lidar_example/simple_lidar_simulator.cpp

# Actualizar CMakeLists.txt para incluir el nodo simulador
RUN echo 'add_executable(simple_lidar_simulator simple_lidar_simulator.cpp) \
\ntarget_link_libraries(simple_lidar_simulator ${catkin_LIBRARIES})' >> /catkin_ws/src/simple_lidar_example/CMakeLists.txt

# Crear archivo URDF para el robot
RUN echo '<?xml version="1.0"?> \
\n<robot name="simple_robot"> \
\n  <link name="base_link"> \
\n    <visual> \
\n      <geometry> \
\n        <box size="0.4 0.4 0.1"/> \
\n      </geometry> \
\n      <material name="blue"> \
\n        <color rgba="0 0 0.8 1"/> \
\n      </material> \
\n    </visual> \
\n  </link> \
\n \
\n  <link name="base_laser"> \
\n    <visual> \
\n      <geometry> \
\n        <cylinder length="0.05" radius="0.05"/> \
\n      </geometry> \
\n      <material name="red"> \
\n        <color rgba="1 0 0 1"/> \
\n      </material> \
\n    </visual> \
\n  </link> \
\n \
\n  <joint name="base_to_laser" type="fixed"> \
\n    <parent link="base_link"/> \
\n    <child link="base_laser"/> \
\n    <origin xyz="0.1 0 0.05"/> \
\n  </joint> \
\n</robot>' > /catkin_ws/src/simple_lidar_example/urdf/simple_robot.urdf

# Crear configuración de RViz
RUN echo 'Panels: \
\n  - Class: rviz/Displays \
\n    Help Height: 78 \
\n    Name: Displays \
\n    Property Tree Widget: \
\n      Expanded: \
\n        - /Global Options1 \
\n        - /Status1 \
\n        - /RobotModel1 \
\n        - /LaserScan1 \
\n        - /Map1 \
\n      Splitter Ratio: 0.5 \
\n    Tree Height: 719 \
\n  - Class: rviz/Selection \
\n    Name: Selection \
\n  - Class: rviz/Tool Properties \
\n    Expanded: \
\n      - /2D Pose Estimate1 \
\n      - /2D Nav Goal1 \
\n      - /Publish Point1 \
\n    Name: Tool Properties \
\n    Splitter Ratio: 0.588679016 \
\n  - Class: rviz/Views \
\n    Expanded: \
\n      - /Current View1 \
\n    Name: Views \
\n    Splitter Ratio: 0.5 \
\n  - Class: rviz/Time \
\n    Experimental: false \
\n    Name: Time \
\n    SyncMode: 0 \
\n    SyncSource: LaserScan \
\nVisualization Manager: \
\n  Class: "" \
\n  Displays: \
\n    - Alpha: 0.5 \
\n      Cell Size: 1 \
\n      Class: rviz/Grid \
\n      Color: 160; 160; 164 \
\n      Enabled: true \
\n      Line Style: \
\n        Line Width: 0.0299999993 \
\n        Value: Lines \
\n      Name: Grid \
\n      Normal Cell Count: 0 \
\n      Offset: \
\n        X: 0 \
\n        Y: 0 \
\n        Z: 0 \
\n      Plane: XY \
\n      Plane Cell Count: 10 \
\n      Reference Frame: <Fixed Frame> \
\n      Value: true \
\n    - Alpha: 1 \
\n      Class: rviz/RobotModel \
\n      Collision Enabled: false \
\n      Enabled: true \
\n      Links: \
\n        All Links Enabled: true \
\n        Expand Joint Details: false \
\n        Expand Link Details: false \
\n        Expand Tree: false \
\n        Link Tree Style: Links in Alphabetic Order \
\n        base_link: \
\n          Alpha: 1 \
\n          Show Axes: false \
\n          Show Trail: false \
\n          Value: true \
\n        base_laser: \
\n          Alpha: 1 \
\n          Show Axes: false \
\n          Show Trail: false \
\n          Value: true \
\n      Name: RobotModel \
\n      Robot Description: robot_description \
\n      TF Prefix: "" \
\n      Update Interval: 0 \
\n      Value: true \
\n      Visual Enabled: true \
\n    - Alpha: 1 \
\n      Autocompute Intensity Bounds: true \
\n      Autocompute Value Bounds: \
\n        Max Value: 10 \
\n        Min Value: -10 \
\n        Value: true \
\n      Axis: Z \
\n      Channel Name: intensity \
\n      Class: rviz/LaserScan \
\n      Color: 255; 255; 255 \
\n      Color Transformer: Intensity \
\n      Decay Time: 0 \
\n      Enabled: true \
\n      Invert Rainbow: false \
\n      Max Color: 255; 255; 255 \
\n      Max Intensity: 4096 \
\n      Min Color: 0; 0; 0 \
\n      Min Intensity: 0 \
\n      Name: LaserScan \
\n      Position Transformer: XYZ \
\n      Queue Size: 10 \
\n      Selectable: true \
\n      Size (Pixels): 3 \
\n      Size (m): 0.00999999978 \
\n      Style: Points \
\n      Topic: /scan \
\n      Unreliable: false \
\n      Use Fixed Frame: true \
\n      Use rainbow: true \
\n      Value: true \
\n    - Alpha: 0.7 \
\n      Class: rviz/Map \
\n      Color Scheme: map \
\n      Draw Behind: false \
\n      Enabled: true \
\n      Name: Map \
\n      Topic: /map \
\n      Unreliable: false \
\n      Use Timestamp: false \
\n      Value: true \
\n    - Class: rviz/TF \
\n      Enabled: true \
\n      Frame Timeout: 15 \
\n      Frames: \
\n        All Enabled: true \
\n        base_laser: \
\n          Value: true \
\n        base_link: \
\n          Value: true \
\n        map: \
\n          Value: true \
\n        odom: \
\n          Value: true \
\n      Marker Scale: 1 \
\n      Name: TF \
\n      Show Arrows: true \
\n      Show Axes: true \
\n      Show Names: true \
\n      Tree: \
\n        map: \
\n          odom: \
\n            base_link: \
\n              base_laser: \
\n                {} \
\n      Update Interval: 0 \
\n      Value: true \
\n  Enabled: true \
\n  Global Options: \
\n    Background Color: 48; 48; 48 \
\n    Default Light: true \
\n    Fixed Frame: map \
\n    Frame Rate: 30 \
\n  Name: root \
\n  Tools: \
\n    - Class: rviz/Interact \
\n      Hide Inactive Objects: true \
\n    - Class: rviz/MoveCamera \
\n    - Class: rviz/Select \
\n    - Class: rviz/FocusCamera \
\n    - Class: rviz/Measure \
\n    - Class: rviz/SetInitialPose \
\n      Topic: /initialpose \
\n    - Class: rviz/SetGoal \
\n      Topic: /move_base_simple/goal \
\n    - Class: rviz/PublishPoint \
\n      Single click: true \
\n      Topic: /clicked_point \
\n  Value: true \
\n  Views: \
\n    Current: \
\n      Class: rviz/Orbit \
\n      Distance: 10 \
\n      Enable Stereo Rendering: \
\n        Stereo Eye Separation: 0.0599999987 \
\n        Stereo Focal Distance: 1 \
\n        Swap Stereo Eyes: false \
\n        Value: false \
\n      Focal Point: \
\n        X: 0 \
\n        Y: 0 \
\n        Z: 0 \
\n      Focal Shape Fixed Size: true \
\n      Focal Shape Size: 0.0500000007 \
\n      Invert Z Axis: false \
\n      Name: Current View \
\n      Near Clip Distance: 0.00999999978 \
\n      Pitch: 0.785398006 \
\n      Target Frame: <Fixed Frame> \
\n      Value: Orbit (rviz) \
\n      Yaw: 0.785398006 \
\n    Saved: ~' > /catkin_ws/src/simple_lidar_example/rviz/slam.rviz

# Crear archivo launch para ejecutar SLAM con datos simulados y RViz
RUN echo '<launch> \
\n  <!-- Cargar descripción del robot --> \
\n  <param name="robot_description" textfile="$(find simple_lidar_example)/urdf/simple_robot.urdf" /> \
\n \
\n  <!-- Nodo simulador de LIDAR y movimiento del robot --> \
\n  <node pkg="simple_lidar_example" type="simple_lidar_simulator" name="robot_simulator" output="screen"/> \
\n \
\n  <!-- SLAM: Gmapping --> \
\n  <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen"> \
\n    <param name="odom_frame" value="odom"/> \
\n    <param name="base_frame" value="base_link"/> \
\n    <param name="map_frame" value="map"/> \
\n    <param name="map_update_interval" value="1.0"/> \
\n    <param name="maxUrange" value="5.0"/> \
\n    <param name="sigma" value="0.05"/> \
\n    <param name="kernelSize" value="1"/> \
\n    <param name="lstep" value="0.05"/> \
\n    <param name="astep" value="0.05"/> \
\n    <param name="iterations" value="5"/> \
\n    <param name="lsigma" value="0.075"/> \
\n    <param name="ogain" value="3.0"/> \
\n    <param name="minimumScore" value="50"/> \
\n    <remap from="scan" to="scan"/> \
\n  </node> \
\n \
\n  <!-- Guardar el mapa periódicamente --> \
\n  <node pkg="map_server" type="map_saver" name="map_saver" args="-f /tmp/my_map" output="screen"> \
\n    <param name="map_save_interval" value="10.0"/> \
\n  </node> \
\n \
\n  <!-- Convertir URDF a TF --> \
\n  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" /> \
\n \
\n  <!-- Controlador del teclado para mover el robot --> \
\n  <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="teleop" output="screen"/> \
\n \
\n  <!-- RViz para visualización --> \
\n  <node pkg="rviz" type="rviz" name="rviz" args="-d $(find simple_lidar_example)/rviz/slam.rviz" /> \
\n</launch>' > /catkin_ws/src/simple_lidar_example/launch/slam_with_rviz.launch

# Compilar el espacio de trabajo
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    catkin_make"

# Script de inicio
RUN echo '#!/bin/bash \
\nsource /opt/ros/noetic/setup.bash \
\nsource /catkin_ws/devel/setup.bash \
\necho "Iniciando sistema SLAM con interfaz gráfica..." \
\necho "El mapa se guardará en /tmp/my_map.pgm y /tmp/my_map.yaml" \
\necho "Usa el nodo teleop_twist_keyboard para mover el robot. Instrucciones:" \
\necho "  - w/x: mover adelante/atrás" \
\necho "  - a/d: girar izquierda/derecha" \
\necho "  - s: detener" \
\nroslaunch simple_lidar_example slam_with_rviz.launch' > /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
```

### Componentes del Sistema

1. **Simulador de Robot y LIDAR**: Un nodo de ROS que simula:
   - El movimiento del robot en un entorno virtual (habitación cuadrada)
   - Datos de un sensor LIDAR que detecta distancias a las paredes
   - Publicación de transformaciones y datos del sensor

2. **Sistema SLAM**: Utiliza gmapping para:
   - Procesar los datos del LIDAR
   - Crear un mapa del entorno
   - Localizar al robot dentro del mapa

3. **Visualización y Control**:
   - Visualización 3D usando RViz
   - Control del robot mediante teclado

# Instrucciones de Uso

## Configuración Inicial

1. Cree un nuevo directorio para su proyecto:

```bash
mkdir slam-robot-simulation
cd slam-robot-simulation
```

2. Descargue el Dockerfile proporcionado:

```bash
# Cree un archivo Dockerfile
touch Dockerfile
# Copie todo el contenido del Dockerfile compartido en este documento al archivo creado
# El Dockerfile comienza con "FROM ros:noetic" y termina con 'ENTRYPOINT ["/entrypoint.sh"]'
```

## Compilación del Contenedor

Una vez que tenga el Dockerfile en su directorio:

```bash
# Construir la imagen Docker
docker build -t slam-robot-simulation .
```

**Nota**: La primera compilación puede tardar varios minutos ya que descarga todas las dependencias necesarias.

## Ejecución del Simulador

```bash
# Permitir conexiones al servidor X
xhost +local:docker

# Ejecutar el contenedor
docker run -it --rm \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  slam-robot-simulation
```

## Control del Robot

Una vez que el sistema está en funcionamiento:

1. La ventana de RViz mostrará el robot (cubo azul con cilindro rojo), los rayos del sensor LIDAR y el mapa que se está generando.
2. Use el terminal donde inició el controlador de teclado para mover el robot:
   * `i`: Avanzar
   * `m`: Retroceder
   * `j`: Girar a la izquierda
   * `l`: Girar a la derecha
   * `k`: Detener el robot
   * `u`, `o`, `n`: Movimientos diagonales

## Salida del Sistema

* El mapa generado se guardará automáticamente en `/tmp/my_map.pgm` y `/tmp/my_map.yaml` dentro del contenedor
* Para guardar el mapa fuera del contenedor, modifique el comando docker run para montar un volumen local

## Estructura del Dockerfile

El Dockerfile realiza los siguientes pasos:

1. Utiliza la imagen base de ROS Noetic
2. Instala las dependencias necesarias (gmapping, rviz, etc.)
3. Configura un espacio de trabajo de catkin
4. Crea un paquete personalizado con:
   * Un simulador de robot y LIDAR
   * Un modelo URDF del robot
   * Configuración de RViz
   * Archivo de lanzamiento para SLAM
5. Compila el espacio de trabajo
6. Configura un script de entrada que inicia todo el sistema

## Mejoras Futuras

1. **Mejoras del Entorno de Simulación**:
   * Añadir obstáculos dinámicos
   * Implementar diferentes diseños de habitaciones/entornos
   * Simular ruido y errores en los sensores para mayor realismo

2. **Mejoras del Robot**:
   * Añadir más sensores (cámara, IMU, codificadores)
   * Implementar cinemática más compleja (robot diferencial, Ackermann)
   * Añadir un modelo 3D más detallado

3. **Algoritmos Avanzados**:
   * Implementar planificación de trayectorias
   * Añadir navegación autónoma
   * Explorar otros algoritmos SLAM (Cartographer, Hector SLAM)

4. **Interfaz de Usuario**:
   * Desarrollar una interfaz web para controlar el robot
   * Añadir visualización de estadísticas en tiempo real
   * Implementar grabación y reproducción de sesiones

5. **Integración y Extensibilidad**:
   * Permitir la carga de mapas predefinidos
   * Añadir APIs para control programático
   * Implementar comunicación con sistemas externos

## Solución de Problemas

### Problemas con la Visualización

Si tiene problemas para mostrar la interfaz gráfica:

```bash
# Verifique que el servidor X está permitiendo conexiones
xhost + 

# Para usuarios de WSL2, asegúrese de tener un servidor X instalado en Windows
# Para usuarios de Mac, asegúrese de tener XQuartz instalado y configurado
```

### Problemas de Permisos

Si encuentra errores de permisos al acceder a los archivos:

```bash
# Ejecute el contenedor con su UID/GID
docker run -it --rm \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --user $(id -u):$(id -g) \
  slam-robot-simulation
```
