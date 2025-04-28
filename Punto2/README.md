# Brazo Robótico de Dos Articulaciones - Simulación con Docker

## Descripción

Este proyecto simula el comportamiento de un brazo robótico de dos articulaciones utilizando la biblioteca `pybullet` en Python. La simulación permite controlar las posiciones de las articulaciones mediante deslizadores en una interfaz gráfica, y se puede ajustar la velocidad de simulación y pausar o continuar la ejecución. El proyecto está configurado para ejecutarse en un contenedor Docker, lo que facilita la gestión de dependencias y la portabilidad del entorno de desarrollo.

## Archivos del Proyecto

### 1. `BrazoRobotico.py`
El archivo `BrazoRobotico.py` es el script principal que utiliza la biblioteca `pybullet` para simular el comportamiento del brazo robótico de dos articulaciones. Este archivo realiza las siguientes acciones:
- **Carga el modelo URDF** del robot y el entorno de simulación.
- **Controla las articulaciones** del robot utilizando deslizadores en la interfaz gráfica para ajustar su posición.
- Permite **ajustar la velocidad de simulación** a través de un parámetro.
- Incluye un **botón de pausa/continuar** para controlar la ejecución de la simulación.

### 2. `two_joint_robot_custom.urdf`
El archivo `two_joint_robot_custom.urdf` describe el modelo físico del robot, incluyendo:
- **Links**: El robot tiene tres enlaces: uno para la base, uno para el primer brazo y otro para el segundo brazo.
- **Joints**: Las articulaciones del robot son de tipo "revoluta" (giran alrededor de un eje), y permiten el movimiento de los enlaces.
- **Materiales y visualización**: Cada parte del robot tiene una representación visual y una configuración de colisión.

### 3. `Dockerfile`
El archivo `Dockerfile` se utiliza para crear un contenedor Docker que contiene todas las dependencias necesarias para ejecutar el proyecto. Este archivo realiza las siguientes acciones:
- **Selecciona una imagen base** de Python 3.8.
- **Instala las dependencias necesarias** como `pybullet` y `numpy`.
- **Copia los archivos del proyecto** al contenedor.
- **Ejecuta el script principal** cuando se inicie el contenedor.

## Pasos Realizados

### 1. Creación de los Archivos Principales

#### **a) `BrazoRobotico.py`**
Este archivo es el encargado de manejar la simulación del brazo robótico. Utiliza `pybullet` para crear una simulación del robot de dos articulaciones y permitir su control a través de deslizadores. La simulación puede ser pausada o continuada, y la velocidad de simulación es ajustable.

#### **b) `two_joint_robot_custom.urdf`**
Este archivo URDF describe el modelo del robot, incluyendo sus enlaces (links) y articulaciones (joints). La estructura física del robot es definida aquí para que `pybullet` pueda simular el comportamiento del robot en el entorno.

#### **c) `Dockerfile`**
Creamos el `Dockerfile` para configurar el entorno de desarrollo de manera portátil. Este archivo prepara el contenedor Docker para ejecutar el proyecto de manera sencilla y rápida.

### 2. Creación y Configuración del Dockerfile

El archivo `Dockerfile` es el encargado de definir el entorno de ejecución del proyecto dentro de un contenedor Docker. Aquí se especifica:
- Usar una imagen base de **Python 3.8**.
- Instalar las dependencias necesarias para el proyecto, como `pybullet` y `numpy`.
- Establecer el directorio de trabajo dentro del contenedor.
- Copiar los archivos del proyecto dentro del contenedor y ejecutar el script principal cuando se inicie el contenedor.

## Contenido de los Archivos

### 1. `BrazoRobotico.py`

```python
import pybullet as p
import pybullet_data
import time
import numpy as np
import os

# Configurar la ruta al archivo URDF
robot_urdf_path = "two_joint_robot_custom.urdf"

# Verificar que el archivo existe
if not os.path.exists(robot_urdf_path):
    print(f"ERROR: No se encuentra el archivo {robot_urdf_path}")
    print("Asegúrate de guardar el archivo URDF en la misma carpeta que este script")
    input("Presiona Enter para salir...")
    exit()

# Inicializar PyBullet
p.connect(p.GUI)  # Modo GUI
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

# Configurar la física y la cámara
p.setGravity(0, 0, -9.8)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=50, cameraPitch=-35, cameraTargetPosition=[0, 0, 0.5])

# Cargar el plano y el robot
planeId = p.loadURDF("plane.urdf")
print(f"Intentando cargar robot desde: {robot_urdf_path}")
robotId = p.loadURDF(robot_urdf_path, [0, 0, 0.1], useFixedBase=True)
print(f"Robot cargado con ID: {robotId}")

# Configurar los deslizadores para los joints
joint1_slider = p.addUserDebugParameter("Joint 1", -np.pi, np.pi, 0)
joint2_slider = p.addUserDebugParameter("Joint 2", -np.pi, np.pi, 0)

# Añadir un parámetro para controlar la velocidad de simulación
speed_control = p.addUserDebugParameter("Velocidad", 0.1, 10.0, 1.0)

# Añadir un botón para pausa
pause_button = p.addUserDebugParameter("Pausar/Continuar", 1, 0, 1)
last_pause_value = 1

print("Simulación iniciada. Use los deslizadores para controlar el robot.")
print("Para mantener la ventana abierta, NO cierre este terminal.")

try:
    # Mantener la simulación en ejecución
    while True:
        # Verificar si se presionó el botón de pausa
        current_pause_value = p.readUserDebugParameter(pause_button)
        if current_pause_value != last_pause_value:
            print("Simulación pausada/continuada")
        last_pause_value = current_pause_value

        # Leer valores de los deslizadores
        joint1_value = p.readUserDebugParameter(joint1_slider)
        joint2_value = p.readUserDebugParameter(joint2_slider)
        speed = p.readUserDebugParameter(speed_control)

        # Controlar las articulaciones
        p.setJointMotorControl2(robotId, 0, p.POSITION_CONTROL, targetPosition=joint1_value, force=500)
        p.setJointMotorControl2(robotId, 2, p.POSITION_CONTROL, targetPosition=joint2_value, force=500)

        # Avanzar la simulación
        p.stepSimulation()
        time.sleep(0.01 / speed)  # Ajusta la velocidad de simulación

except KeyboardInterrupt:
    print("Simulación interrumpida por el usuario")
finally:
    # Asegurarse de que la ventana se mantenga abierta hasta que el usuario presione Enter
    input("\nSimulación finalizada. Presiona Enter para cerrar completamente...")
    p.disconnect()
```
### 2. `two_joint_robot_custom.urdf`

```xml
<?xml version="1.0"?>
<robot name="two_joint_robot">
    <link name="base_link">
        <visual>
            <geometry>
                <cylinder length="0.1" radius="0.2"/>
            </geometry>
            <material name="blue">
                <color rgba="0 0 0.8 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <cylinder length="0.1" radius="0.2"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="1"/>
            <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
        </inertial>
    </link>

    <link name="link1">
        <visual>
            <geometry>
                <box size="0.1 0.1 0.5"/>
            </geometry>
            <origin xyz="0 0 0.25"/>
            <material name="red">
                <color rgba="0.8 0 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="0.1 0.1 0.5"/>
            </geometry>
            <origin xyz="0 0 0.25"/>
        </collision>
        <inertial>
            <mass value="1"/>
            <origin xyz="0 0 0.25"/>
            <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.01"/>
        </inertial>
    </link>

    <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="link1"/>
        <origin xyz="0 0 0.05"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3.14" upper="3.14" effort="100" velocity="5"/>
    </joint>

    <link name="link2">
        <visual>
            <geometry>
                <box size="0.1 0.1 0.5"/>
            </geometry>
            <origin xyz="0 0 0.25"/>
            <material name="green">
                <color rgba="0 0.8 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="0.1 0.1 0.5"/>
            </geometry>
            <origin xyz="0 0 0.25"/>
        </collision>
        <inertial>
            <mass value="1"/>
            <origin xyz="0 0 0.25"/>
            <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.01"/>
        </inertial>
    </link>

    <joint name="joint2" type="revolute">
        <parent link="link1"/>
        <child link="link2"/>
        <origin xyz="0 0 0.5"/>
        <axis xyz="0 1 0"/>
        <limit lower="-3.14" upper="3.14" effort="100" velocity="5"/>
    </joint>

    <link name="end_effector">
        <visual>
            <geometry>
                <sphere radius="0.05"/>
            </geometry>
            <material name="white">
                <color rgba="1 1 1 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <sphere radius="0.05"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="0.1"/>
            <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
        </inertial>
    </link>

    <joint name="joint_end_effector" type="fixed">
        <parent link="link2"/>
        <child link="end_effector"/>
        <origin xyz="0 0 0.5"/>
    </joint>
</robot>
```

### 3. `Dockerfile`
```Dockerfile
# Usar una imagen base de Python 3
FROM python:3.8-slim

# Instalar dependencias necesarias
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libglu1-mesa \
    && rm -rf /var/lib/apt/lists/*

# Instalar las librerías de Python necesarias
RUN pip install --upgrade pip
RUN pip install pybullet numpy

# Establecer el directorio de trabajo
WORKDIR /app

# Copiar los archivos del proyecto al contenedor
COPY . /app

# Comando para ejecutar el script
CMD ["python", "BrazoRobotico.py"]
```

# Creación y Ejecución del Contenedor Docker
Para ejecutar el proyecto en Docker, se deben seguir los suguientes pasos:

Paso 1: Construir la imagen Docker
Se construye la imagen Docker utilizando el siguiente comando:

```
docker build -t brazo_robotico .
```

Paso 2: Permitir el acceso a la GUI
Para ejecutar aplicaciones gráficas dentro de Docker, se necesita que el contenedor acceda a la pantalla local. Para hacerlo, se ejecuta el siguiente comando:

```
xhost +local:root
```

Paso 3: Ejecutar el contenedor con acceso a la GUI
Ahora, se ejecuta el contenedor con el siguiente comando, lo que permite que el contenedor acceda a la GUI y ejecute el script BrazoRobotico.py:

```
xhost +local:root
docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
```

Esto inicia la simulación del brazo robótico, y se puede interactuar con el robot a través de la interfaz gráfica.

4. Detener el Contenedor
Para detener el contenedor, simplemente se debe presiona Ctrl+C en la terminal donde se está ejecutando. También se puede ejecutar el siguiente comando para detener el contenedor manualmente:

```
docker-compose down
```
Mejoras Futuras:

- Control de velocidad individual por articulación: Actualmente, la velocidad es global, pero se podría mejorar para permitir ajustar la velocidad de cada articulación por separado.

- Integración con ROS: Integrar el brazo robótico con el sistema ROS (Robot Operating System) para un control más avanzado y la posibilidad de interactuar con otros sistemas robóticos.

- Sensores adicionales: Agregar sensores como cámaras o LiDAR para hacer el robot más interactivo y autónomo.

- Algoritmos de control avanzados: Implementar controladores PID o incluso redes neuronales para controlar el movimiento del robot de forma más precisa.
