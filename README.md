# TFM_MiguelLerinAlonso

En este repositorio se guarda todo el software desarrollando para poder realizar una instalación rápida del sistema.

> **IMPORTANTE**
>
>En este archivo README tan sólo se van a describir aspectos como la composición del repositorio o la guía básica de descarga e instalación. Para leer la documentación especializada de cada paquete, se recomienda leer su respectivo artículo en la [wiki](https://github.com/Miguel-LA/TFM_MiguelLerinAlonso/wiki) disponible en la página de Github.

# Composición del repositorio
El repositorio se divide en las siguientes secciones, representadas cada una por su propia carpeta:

1. **workspace:** Este directorio sirve epara establecer un espacio de trabajo ROS2 con los equipos róbotivos del Laboratorio de Ingeniería de Fabricación de la ETSII-UPM. Se ha diseñado para que se le puedan añadir otros sistemas en el futuro como el AGV MiR10 o el robot Kuka.

    1.1. **ros_ur_driver:** Este directorio guarda el espacio de trabajo propio del robot UR10 utilizado en este TFM.

2. **wiki:** Esta carpeta alberga los ficheros de documentación del proyecto. Se guardan los mismos archivos que en la wiki pero se ponen en copia local por si en algún momento se debe dejar el acceso al repo como privado.

# Descarga e instalación del repositorio
## Software previo
El repositorio utiliza ROS2 Humble como versión de control. Para la comunicación con el robot, el cálculo de trayectorias y la visualización de movimientos se utiliza el software Moveit2. El nombre de la lista contine un enlace a la documentación oficial. Para aquel software disponible para otras versiones de ROS2 se incluya la versión correspondiente a su rama *main* de su respectivo repositorio. También se indica una referencia al artículo propio de instalación de la wiki.

- [ROS2 Humble](https://docs.ros.org/en/humble/index.html): Es la versión de ROS2 utilizada en este trabajo. En este proyecto se ha optado por la instalación en el [sistema operativo Ubuntu 22 con gestor de paquetes de Debian](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). Para otro tipo de instalación se recomienda consultar la documentación oficial. Más desarrollo en el artículo de la wiki.
- [Moveit2](https://moveit.picknik.ai/main/index.html): Es un software de código abierto utilizado para simular el robot y calcular las trayectorias efectuadas por el mismo. Más desarrollo en el artículo de la wiki.
- [ROS2 UR Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver): Controlador de Universal Robots para actuar como una interfaz entre el entorno de ROS2 de nuestro equipo y el robot UR al que nos deseemos conectar.

## Instalación del repositorio

Si ya se tiene bien instalado y configurado el software de la sección anterior, se puede copiar y pegar el siguiente bloque de código en la terminal. El proceso de instalación paso a paso se encuentra descrito en la wiki.   

````bash
source /opt/ros/humble/setup.bash
git clone -b main https://github.com/Miguel-LA/TFM_MiguelLerinAlonso.git
cd ./TFM_MiguelLerinAlonso/workspace/ros_ur_driver/
vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver-not-released.${ROS_DISTRO}.repos
rosdep update
rosdep install --ignore-src --from-paths src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Obtener la ruta completa del directorio actual
REPO_DIR=$(pwd)

# Agregar las instrucciones a .bashrc para cargar automáticamente al iniciar una terminal
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source $REPO_DIR/install/setup.bash" >> ~/.bashrc
````

Si durante la fase de compilación con `colcon build` ocurre algún tipo de error se debe actualizar el sistema operativo.

````bash
sudo apt update
sudo apt upgrade
````

> **IMPORTANTE**
>
>Para la solución de posibles problemas de instalación se recomienda visitar la [página de la wiki](https://github.com/Miguel-LA/TFM_MiguelLerinAlonso/wiki/Instalacion_repo) del repositorio.