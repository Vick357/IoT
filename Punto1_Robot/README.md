# Sistema de Simulación de Robot con Docker, ROS2 Humble y Gazebo Harmonic

Este proyecto proporciona un entorno completo de simulación robótica basado en Docker, utilizando ROS2 Humble y Gazebo Harmonic. Esta configuración permite ejecutar simulaciones en un entorno replicable, portable y aislado, ideal para pruebas de robots sin necesidad de configuraciones complejas en el sistema anfitrión.

## Descripción

El sistema utiliza:
- **ROS2 Humble**: Una distribución LTS (soporte hasta 2027) del sistema operativo robótico que facilita la creación y control de aplicaciones robóticas
- **Gazebo Harmonic**: Un simulador de física 3D que permite probar robots en entornos virtuales
- **Docker**: Para encapsular todo el entorno y garantizar compatibilidad entre sistemas

Este entorno permite simular sensores, actuadores y entornos físicos de manera realista, proporcionando un sandbox controlado para desarrollo y pruebas robóticas.

## Estructura de Archivos

El proyecto contiene los siguientes archivos principales:

## Archivos del Proyecto

### Dockerfile

El Dockerfile configura la imagen base con ROS2 Humble y todas las dependencias necesarias para el entorno de simulación:

```dockerfile
FROM ros:humble
ENV DEBIAN_FRONTEND=noninteractive

# ROS2
# Create a workspace
WORKDIR /ros2_humble_ws/src

# install ros package
RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/*

# Gazebo Fortress
# Set the working directory
WORKDIR /root

# Install the necessary packages
RUN apt-get update && apt-get install -y \ 
    lsb-release \ 
    wget \ 
    gnupg \
    x11-apps \
    libxext-dev \
    libxrender-dev \
    libxtst-dev 

RUN sudo apt-get install  -y ros-${ROS_DISTRO}-ros-gz

RUN apt install -y vim
RUN apt install -y htop
RUN apt install -y nvtop 

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_humble_ws/install/setup.bash" >> ~/.bashrc

# Set Gazebo models path environment variable
# see https://gazebosim.org/api/sim/8/resources.html
# Modify here to add your own models path
RUN echo "export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:/ros2_humble_ws/src/" >> ~/.bashrc

# An alias to source the bashrc file
RUN echo "alias sb='source ~/.bashrc'" >> ~/.bashrc

CMD [ "/ros2_humble_ws/src/init.sh"]
```

**Análisis del Dockerfile:**

- **Base**: Utiliza la imagen oficial de ROS2 Humble (`FROM ros:humble`)
- **Entorno de trabajo**: Configura un workspace en `/ros2_humble_ws/src`
- **Paquetes ROS2**: Instala nodos de demostración en C++ y Python
- **Dependencias de Gazebo**: Instala paquetes necesarios para Gazebo y aplicaciones gráficas X11
- **Integración ROS-Gazebo**: Instala `ros-gz` para la interconexión entre ROS2 y Gazebo
- **Herramientas**: Instala utilidades como vim (editor), htop (monitor de sistema) y nvtop (monitor de GPU)
- **Configuración de entorno**: Configura automáticamente el entorno ROS2 y Gazebo al iniciar una shell
- **Script de inicio**: Configura el script `init.sh` como punto de entrada

### docker-compose.yml

Este archivo facilita la creación y ejecución del contenedor Docker:

```yaml
version: '3.8'
services:
  ros2_humble_gazebo:
    build:
      context: .
      dockerfile: Dockerfile
    container_name: ros2_humble_gazebo_harmonic
    network_mode: "host"
    environment:
      DISPLAY: ${DISPLAY}
    volumes:
      - /var/run/docker.sock:/var/run/docker.sock
      - /tmp/.X11-unix:/tmp/.X11-unix
      - ./ros2_humble_ws:/ros2_humble_ws/src
```

**Análisis del docker-compose.yml:**

- **Versión**: Utiliza la versión 3.8 de Docker Compose
- **Servicio**: Define un servicio llamado "ros2_humble_gazebo"
- **Construcción**: Indica construir la imagen a partir del Dockerfile en el contexto actual
- **Nombre del contenedor**: Asigna el nombre "ros2_humble_gazebo_harmonic" al contenedor
- **Red**: Usa el modo de red "host" para compartir la red con el sistema anfitrión
- **Variables de entorno**: Comparte la variable DISPLAY para permitir interfaces gráficas
- **Volúmenes**:
  - Socket Docker: Permite comunicación con el demonio Docker
  - Sockets X11: Habilita interfaces gráficas
  - Directorio de trabajo: Monta el directorio local `./ros2_humble_ws` en `/ros2_humble_ws/src` dentro del contenedor

### init.sh

Script de inicialización que se ejecuta al iniciar el contenedor:

```bash
#!/bin/bash
# This script is run when the container is started!
echo "ROS2 Humble and Gazebo Harmonic Docker running!"
echo "
    ____  ____  ________      ____             __            
   / ** \/ ** \/ ___/__ \    / ** \**__  _____/ /_____  _____
  / /_/ / / / /\__ \__/ /   / / / / ** \/ **_/ //_/ * \/ *__/
 / *, */ /_/ /___/ / __/   / /_/ / /_/ / /__/ ,< /  __/ /    
/_/ |_|\____//____/____/  /_____/\____/\___/_/|_|\___/_/     
"
sleep infinity
```

**Análisis del init.sh:**

- **Mensaje de bienvenida**: Muestra un banner ASCII art con información sobre el entorno
- **Ejecución continua**: La instrucción `sleep infinity` mantiene el contenedor en ejecución indefinidamente, lo que es crucial para que el contenedor no se detenga después de mostrar el mensaje inicial

## Requisitos Previos

- Docker Engine
- Docker Compose
- Sistema X Window (para interfaces gráficas)
- Git

## Instalación y Configuración

1. Clone el repositorio:
   ```bash
   mkdir Punto1_Robot
   cd Punto1_Robot
   git init
   git remote add origin https://github.com/Vick357/IoT.git
   git config core.sparseCheckout true
   echo "Punto1_Robot/*" >> .git/info/sparse-checkout
   git pull origin main
   ```

2. Navegue al directorio del proyecto:
   ```bash
   cd ~/Prueba/IoT/Punto1_Robot
   ```

3. Construya la imagen Docker:
   ```bash
   docker-compose build
   ```

4. Otorgue permisos de ejecución al script de inicialización:
   ```bash
   chmod +x ros2_humble_ws/init.sh
   ```
   
5. Inicie el contenedor:
   ```bash
   docker-compose up
   ```
   
## Ejecución - En una terminal nueva

1. Configure los permisos de X11 para permitir interfaces gráficas:
   ```bash
   xhost +local:root
   ```

2. Para acceder al contenedor en ejecución:
   ```bash
   docker ps                            # Identifique el ID del contenedor
   docker exec -it <CONTAINER_ID> bash  # Sustituya <CONTAINER_ID> con el ID real
   ```

4. Para iniciar Gazebo dentro del contenedor:
   ```bash
   ign gazebo
   ```

## Funcionalidades del Sistema

### ROS2 Humble
ROS2 (Robot Operating System 2) es un conjunto de bibliotecas y herramientas para el desarrollo de aplicaciones robóticas. La distribución Humble Hawksbill es una versión LTS (Long Term Support) lanzada en mayo de 2022 y con soporte hasta 2027. Proporciona:

- Arquitectura distribuida basada en nodos
- Comunicación mediante tópicos, servicios y acciones
- Middleware DDS para comunicaciones en tiempo real
- Gran variedad de paquetes y herramientas

### Gazebo Harmonic
Gazebo es un simulador 3D que permite probar robots en entornos virtuales con características como:

- Simulación de fuerzas físicas (gravedad, fricción)
- Simulación de sensores (cámaras, LIDAR, etc.)
- Control de actuadores
- Integración con ROS2 mediante el paquete ros-gz

### Componentes de la Simulación Robótica
El sistema de simulación puede incluir:

- **Modelos del robot (URDF/SDF)**: Describen la estructura física del robot
- **Controladores de articulaciones**: Permiten enviar comandos a las articulaciones
- **Nodos ROS2**: Programas que controlan al robot y procesan datos
- **Herramientas de visualización**: Gazebo y RViz2 para observar el robot y sus datos

## Posibles Mejoras

1. **Automatización de configuración inicial**:
   - Agregar un script que configure automáticamente los permisos X11 y otros requisitos previos.

2. **Inclusión de modelos de robot predefinidos**:
   - Añadir modelos URDF/SDF de ejemplo para empezar rápidamente.

3. **Integración con RViz2**:
   - Añadir configuraciones para visualizar datos de sensores y transformaciones.

4. **Mapas y entornos**:
   - Incluir entornos predefinidos para simulación (habitaciones, laberintos, etc.).

5. **Tutoriales integrados**:
   - Añadir ejemplos paso a paso para aprender a usar el sistema.

6. **Soporte para GPU**:
   - Mejorar la configuración para aprovechar aceleración por GPU en la simulación.

7. **Interfaz web**:
   - Implementar acceso vía navegador para eliminar dependencia de X11.

8. **Integración CI/CD**:
   - Añadir pruebas automatizadas y pipeline de integración continua.

9. **Optimización de imagen Docker**:
   - Reducir el tamaño de la imagen utilizando técnicas multi-stage.

10. **Documentación expandida**:
    - Añadir ejemplos de uso, detalles de la API y guías detalladas.

## Solución de Problemas

Si encuentra problemas con las interfaces gráficas:
- Asegúrese de haber ejecutado `xhost +local:root` antes de iniciar el contenedor
- Verifique que la variable `DISPLAY` esté correctamente configurada
- Compruebe que su sistema X Window esté funcionando correctamente

Si el contenedor se detiene inmediatamente:
- Verifique que el script `init.sh` tenga permisos de ejecución
- Asegúrese de que el script contenga `sleep infinity` para mantener el contenedor en ejecución

Si Gazebo no se inicia correctamente:
- Verifique que las dependencias de Gazebo estén correctamente instaladas
- Compruebe que la variable de entorno `GZ_SIM_RESOURCE_PATH` esté correctamente configurada
