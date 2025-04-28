# Usa una imagen base con Python y soporte gráfico
FROM python:3.10-slim

# Instala las librerías necesarias
RUN apt-get update && apt-get install -y \
    x11-apps \
    libgl1-mesa-glx \
    libglu1-mesa \
    && rm -rf /var/lib/apt/lists/*

# Instala PyBullet
RUN pip install pybullet numpy

# Crea un directorio de trabajo
WORKDIR /app

# Copia el script y el URDF
COPY BrazoRobotico.py .
COPY two_joint_robot_custom.urdf .

# Comando para ejecutar el script
CMD ["python", "BrazoRobotico.py"]
