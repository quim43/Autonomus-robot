🚀 Navegación Autónoma con ROS2 | ROSNi Robot

Este repositorio documenta el desarrollo de un sistema de navegación autónoma para un robot móvil utilizando ROS2 Humble. Se implementaron técnicas de SLAM, planificación de rutas y control de movimiento para permitir la exploración de entornos desconocidos.

📌 Descripción del Proyecto

El robot utilizado es una plataforma educativa basada en ROSNi, equipada con:
	•	Raspberry Pi 4 con Ubuntu 22.04 y ROS2 Humble.
	•	Arduino Nano para el control de motores e interfaz de usuario.
	•	YDLidar X4 para percepción del entorno con un sensor LIDAR de 360°.
	•	Motores DC con reductora y encoders para navegación precisa.

El objetivo del proyecto es permitir que el robot explore y mapee un entorno de manera autónoma, detectando fronteras de exploración y generando rutas seguras hasta que el área esté completamente mapeada.

🏗️ Arquitectura del Sistema

El sistema está compuesto por varios paquetes ROS2:
	•	rosni2_gazebo → Simulación del robot en Gazebo.
	•	rosni2_description → Modelado del robot en URDF.
	•	frontier_explorer → Algoritmo para detección y selección de fronteras explorables.
	•	global_planner → Planificación global de rutas seguras evitando obstáculos.
	•	local_planner → Control del robot mediante un Control PD, generando comandos de movimiento /cmd_vel.

🔍 Funcionalidades Implementadas

✔️ SLAM en tiempo real con slam_toolbox.
✔️ Exploración autónoma basada en detección de fronteras.
✔️ Planificación de rutas utilizando información del mapa generado.
✔️ Simulación en Gazebo antes de implementación en hardware real.
✔️ Control del robot con ROS2 Topics (/cmd_vel, /scan, /map, etc.).

⚙️ Instalación y Uso

1️⃣ Instalar dependencias:
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox

2️⃣ Clonar el repositorio en tu workspace de ROS2:
cd ~/ros2_ws/src
git clone https://github.com/..
cd ..
colcon build

3️⃣ Ejecutar la simulación en Gazebo:
ros2 launch rosni2_gazebo rosni_slam.launch.py

4️⃣ Iniciar exploración autónoma:
ros2 run frontier_explorer explorer_node

 Objetivo Final

El robot debe ser capaz de explorar de manera completamente autónoma, generando un mapa del entorno y optimizando su navegación en función de los obstáculos y zonas ya exploradas.

El código se ha probado en simulación y posteriormente validado en hardware real. Se seguirán realizando mejoras para optimizar la velocidad y eficiencia del sistema.










