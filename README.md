<p align="center">

  <!-- ---------- TOP IMAGE ---------- -->
  <a href="https://www.cartif.es/en/human-robot-collaboration-pilot-plant-cartifactory/">
    <img src="docs/assets/cartifactory-transparent.png" alt="Top image" height="100"/>
  </a>
  <!-- ---------- CARTIF logo ---------- -->
  <a href="https://www.cartif.es/en/home/">
    <img src="docs/assets/CARTIF_transparente.png" height="100">
  </a>

</p>



# CARTIFactory, Middleware de Interacción Humano-Robot en Industria 5.

[![vulcanexus](https://img.shields.io/badge/Vulcanexus%20Version-Jazzy-%230895CD?style=flat)](https://github.com/eProsima/vulcanexus)
[![ubuntu24](https://img.shields.io/badge/Ubuntu-24.04-%23E95420?style=flat&logo=ubuntu
)](https://releases.ubuntu.com/24.04/)

# Contenidos
- [CARTIFactory, Middleware de Interacción Humano-Robot en Industria 5.](#cartifactory-middleware-de-interacción-humano-robot-en-industria-5)
- [Contenidos](#contenidos)
- [Inicio](#inicio)
    - [1. Construir la imagen](#1-construir-la-imagen)
  - [2 Definir la entrada](#2-definir-la-entrada)
    - [2.1 Sin cámara física: utilizar un rosbag](#21-sin-cámara-física-utilizar-un-rosbag)
      - [Preparar las grabaciones y arrancar](#preparar-las-grabaciones-y-arrancar)
      - [Consultar y reproducir](#consultar-y-reproducir)
    - [2.2 Con cámara física: añadir el acceso USB](#22-con-cámara-física-añadir-el-acceso-usb)
  - [3. Configurar y activar el pipeline en ambos modos](#3-configurar-y-activar-el-pipeline-en-ambos-modos)
  - [4. Cambiar de modo y comprobar el estado](#4-cambiar-de-modo-y-comprobar-el-estado)
  - [5. Red, RViz y servicios FIWARE](#5-red-rviz-y-servicios-fiware)
  - [Nodos de ROS2](#nodos-de-ros2)

# Inicio

El módulo puede recibir datos de una **cámara RealSense conectada por USB** o de un **rosbag grabado previamente**. Se utiliza la misma imagen Docker en ambos casos:

| Modo | Archivos Compose |
| --- | --- |
| Sin cámara física / reproducción de rosbag | `docker-compose.yml` |
| Con cámara RealSense USB | `docker-compose.yml` + `docker-compose-usb.yml` |

Ejecuta los comandos Docker desde la **raíz del repositorio**, en PowerShell, la terminal de VS Code o una terminal Linux con acceso a Docker. Los comandos ROS 2 se ejecutan **dentro del contenedor**.

> [!IMPORTANT]
> El Compose principal monta `./ros_bags:/ros_bags_source:ro` como origen de importación y el volumen persistente `ros_bags_data:/ros_bags` para reproducir. **No debe incluir el montaje de `/dev/bus/usb`**. Ese montaje pertenece exclusivamente a `docker-compose-usb.yml`. De lo contrario, el arranque sin cámara también exigirá el dispositivo USB.
>
> Los Compose arrancan el pipeline de los nodos del ciclo de vida ([*lifecycle* o *managed nodes*](https://design.ros2.org/articles/node_lifecycle.html)). **No lanzan automáticamente el driver de RealSense ni reproducen un rosbag.**

### 1. Construir la imagen

La primera vez, o después de modificar el Dockerfile, las dependencias o el código que se copia y compila dentro de la imagen:

```bash
docker compose -f docker-compose.yml build operator_tracking
```

Docker reutiliza las capas que no han cambiado. No es necesario reconstruir la imagen por conectar o desconectar la cámara, cambiar de modo o añadir nuevas grabaciones a `ros_bags/`.

## 2 Definir la entrada

Como se ha indicado en el apartado anterior, es posible probar este módulo utilizando una cámara Realsense conectada al ordenador (cosa que haremos nosotros en le workshop). Para facilitar la reproducibilidad y que podáis ver como funciona en vuestros esquipos, os proporcionamos una serie de ros bags, que podéis descargar del [siguiente enlace]().

### 2.1 Sin cámara física: utilizar un rosbag

#### Preparar las grabaciones y arrancar

Guarda cada grabación completa en `ros_bags/<nombre_del_bag>/`, dentro del repositorio. Debe incluir `metadata.yaml` y todos los archivos de datos que este referencia (`.mcap` o `.db3`).

El Compose monta dos rutas distintas dentro del contenedor:

| Ruta | Almacenamiento | Uso |
| --- | --- | --- |
| `/ros_bags_source` | Carpeta `./ros_bags` del host, montada en solo lectura. | Origen para importar grabaciones. |
| `/ros_bags` | Volumen persistente de Docker `operator_tracking_bags`. | Lectura y reproducción dentro del almacenamiento Linux de Docker. |

El volumen se crea si todavía no existe y se reutiliza en los siguientes arranques, tanto en modo cámara como en modo rosbag. No es necesario añadir un Compose específico para los bags.

Arranca solo el módulo, reutilizando la imagen construida:

```bash
docker compose -f docker-compose.yml up -d --no-deps --no-build operator_tracking
```
>[!warning] Recuerda
No añadas `docker-compose-usb.yml` ni ejecutes `usbipd attach` para este modo.

El entrypoint importa automáticamente los archivos nuevos desde `/ros_bags_source/` hacia `/ros_bags/` mediante `rsync -a --ignore-existing`, antes de lanzar el pipeline. La primera importación de grabaciones grandes puede tardar varios minutos. Los archivos ya existentes se omiten; si sustituyes un archivo conservando el mismo nombre, no se actualizará automáticamente.

Sigue el arranque desde el host:

```bash
docker compose -f docker-compose.yml logs -f operator_tracking
```

**Espera al mensaje siguiente antes de reproducir un bag:**

```text
[entrypoint] Grabaciones disponibles en /ros_bags.
```

Después aparecerán los logs de los nodos del pipeline. Pulsa `Ctrl+C` para salir del seguimiento de logs; esto no detiene el contenedor.

> [!IMPORTANT]
> Que Docker muestre el contenedor como `Started` o `Running` no significa que la importación haya terminado. Incluso es posible entrar con `docker exec` mientras se copian los bags.
>
> La carpeta y `metadata.yaml` pueden aparecer antes que el archivo de datos definitivo. Un nombre como `.robot_camera_20261020_133206_0.mcap.Fb89Dr` es un archivo temporal de `rsync`. No lo renombres: espera a que termine la copia. Si hay errores o reinicios continuos, revisa los logs.
>
> Las grabaciones copiadas sobreviven a la recreación del contenedor. `docker compose down -v` elimina los volúmenes gestionados por ese Compose, incluidas estas copias. Los originales de `./ros_bags` permanecen en el anfitrión.

#### Consultar y reproducir

Abre una terminal del contenedor:

```bash
docker compose -f docker-compose.yml exec operator_tracking bash
```

Dentro, comprueba la grabación antes de reproducirla:

```bash
ros2 bag info /ros_bags/robot_camera_20261020_133206
```

El siguiente comando se ha probado con esta grabación. Sustituye el nombre de la carpeta por el de tu bag y adapta la lista si sus topics son diferentes:

```bash
ros2 bag play /ros_bags/robot_camera_20261020_133206 \
  --clock \
  --read-ahead-queue-size 500 \
  --start-paused \
  --topics \
    /camera/cameraD455/color/image_raw \
    /camera/cameraD455/color/camera_info \
    /camera/cameraD455/aligned_depth_to_color/image_raw \
    /camera/cameraD455/aligned_depth_to_color/camera_info \
    /ur/tcp_pose \
    /tf \
    /tf_static \
    /robot_description
```

El reproductor arranca **pausado**. Deja esa terminal abierta, prepara el módulo y RViz en otras terminales y pulsa **Espacio** en la terminal del bag para comenzar. Espacio alterna pausa y reproducción; `Ctrl+C` detiene el reproductor.

| Opción | Función |
| --- | --- |
| `--clock` | Publica `/clock` con el tiempo de reproducción para los nodos que tengan `use_sim_time=true`. No cambia los timestamps guardados en los mensajes. |
| `--read-ahead-queue-size 500` | Configura una cola de lectura anticipada de 500 mensajes. No son 500 MB ni 500 fotogramas; el consumo de memoria depende del tamaño de los mensajes. |
| `--start-paused` | Permite preparar los consumidores antes de comenzar la reproducción. |
| `--topics` | Limita la reproducción a los topics enumerados. Reduce el tráfico publicado, aunque el formato de almacenamiento puede requerir leer o descomprimir bloques que contengan otros topics. |

Los topics seleccionados cumplen estas funciones:

| Topics | Uso |
| --- | --- |
| `.../color/image_raw` y `.../color/camera_info` | Imagen RGB para detección y datos de calibración. |
| `.../aligned_depth_to_color/image_raw` y  `.../aligned_depth_to_color/camera_info` | Información de profundidad alineada con el color para la proyección 3D. |
| `/ur/tcp_pose` | Pose TCP grabada, utilizada por la etapa de transformación cámara-base. |
| `/tf` y `/tf_static` | Transformaciones dinámicas y estáticas para situar los elementos en sus sistemas de coordenadas. |
| `/robot_description` | Descripción URDF que puede utilizar el display RobotModel de RViz. |

Este ejemplo no reproduce la nube de puntos ni `/joint_states`. Puedes añadirlos a la lista si los necesitas y están grabados. Los TF grabados ya permiten reproducir las transformaciones del robot, no hace falta lanzar simultáneamente otro publicador de esas mismas transformaciones.

Los nombres de cámara deben coincidir con la configuración del módulo, por defecto `cameraD455`. Si el bag utiliza otros nombres, adapta `CAMERA_NAME` o los remapeos.

Por defecto se realiza **una sola pasada** y el comando termina al finalizar la grabación. Para repetirla, añade `--loop` al comando anterior. En cada vuelta el tiempo de reproducción vuelve al comienzo: comprueba que los nodos, sus buffers y TF toleran ese salto. Si aparecen problemas justo al reiniciar, valida primero una sola pasada.

>[!warning] Rendimiento en la lectura y advertencias
>
>En las pruebas con Windows, un bag MCAP de 6,4 GiB y unos 58 segundos provocaba este aviso al reproducirlo desde una carpeta de Windows montada directamente:
>
>```text
>Message queue starved. Messages will be delayed.
>```
>
>El aviso indica que el reproductor se queda sin mensajes preparados para publicar; **no demuestra por sí solo que el bag esté dañado**. Aumentar la cola y filtrar topics no bastó en esa prueba. La reproducción desde una copia en el almacenamiento Linux de Docker eliminó el aviso y mejoró la fluidez de imagen y TF. Por eso se utiliza el volumen persistente como origen de reproducción.
>
>Reproduce desde **`/ros_bags`**, no desde `/ros_bags_source`. Si el aviso continúa después de completar la importación, revisa la carga del equipo, la lectura/descompresión y los topics seleccionados; no aumentes indefinidamente la cola. El volumen utiliza el disco de datos configurado en Docker Desktop, que no tiene por qué estar en el mismo disco que el repositorio.

> [!NOTE] No mezclar apartados 2.1 y 2.2
> No ejecutes simultáneamente un driver de cámara o un publicador de TCP real que publique los mismos topics que el bag. Para trabajar con `--clock`, configura el tiempo simulado en los consumidores y en RViz como se indica en las secciones siguientes. Revisa también su QoS si faltan mensajes.



### 2.2 Con cámara física: añadir el acceso USB

Conecta la RealSense y cierra las aplicaciones que puedan estar utilizándola. **En Windows con WSL 2**, instala y configura previamente `usbipd-win` siguiendo la [guía de Microsoft](https://learn.microsoft.com/es-es/windows/wsl/connect-usb). Abre Ubuntu y déjalo en ejecución:

```powershell
wsl -d Ubuntu
```

En otra terminal de **PowerShell**, identifica la cámara y conéctala a WSL:

```powershell
usbipd list
usbipd attach --wsl --busid <BUSID>
```

Sustituye `<BUSID>` por el identificador actual de la RealSense. Si figura como `Not shared`, un administrador debe ejecutar primero `usbipd bind --busid <BUSID>`. Si ya figura como `Attached`, no repitas el attach.

Cuando el motor Linux de Docker tenga acceso al USB, arranca el módulo combinando los dos Compose:

```bash
docker compose -f docker-compose.yml -f docker-compose-usb.yml up -d --no-deps --no-build operator_tracking
docker compose -f docker-compose.yml -f docker-compose-usb.yml exec operator_tracking bash
```

Dentro del contenedor, comprueba la visibilidad de la cámara y la existencia de su dispositivo:

```bash
lsusb
ls -l /dev/bus/usb/*/
```

Después lanza el driver. Este perfil utiliza color y profundidad a 640 × 480 y 15 FPS, con alineación y nube de puntos:

```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera \
  camera_name:=cameraD455 \
  device_type:=d455 \
  enable_color:=true \
  enable_depth:=true \
  enable_gyro:=false \
  enable_accel:=false \
  align_depth.enable:=true \
  pointcloud.enable:=true \
  pointcloud.allow_no_texture_points:=false \
  rgb_camera.color_profile:=640x480x15 \
  depth_module.depth_profile:=640x480x15 \
  decimation_filter.enable:=false \
  pointcloud.ordered_pc:=true \
  clip_distance:=2.0 \
  publish_tf:=true
```

Deja abierta esta terminal mientras utilizas la cámara. El nombre del driver debe coincidir con el del pipeline, por defecto `cameraD455`.

> [!WARNING] Problemas en la ruta /dev/bus/usb
> Si aparece `bind source path does not exist: /dev/bus/usb`, el fallo se produce **al crear el contenedor**, no al construir la imagen. Comprueba el acceso USB desde el motor Linux de Docker. Que Windows o Ubuntu detecten la cámara no confirma por sí solo el acceso desde Docker.
>
> No crees una carpeta vacía para ocultar el error. Para trabajar con un rosbag, utiliza únicamente el Compose principal.

## 3. Configurar y activar el pipeline en ambos modos

Con el bag reproduciéndose o la cámara publicando, abre **otra terminal del contenedor** desde el host:

```bash
docker exec -it operator_tracking bash
```

>[!note] En caso de estar utilizando los ros bags
**En modo rosbag con `--clock`**, establece primero el tiempo simulado en los cinco nodos, antes de configurarlos y activarlos:
>
>```bash
>ros2 param set /hand_mediapipe_node use_sim_time true
>ros2 param set /hand_landmarks_3d_node use_sim_time true
>ros2 param set /hand_palm_plane_node use_sim_time true
>ros2 param set /hand_cam_to_base_node use_sim_time true
>ros2 param set /orchestrator use_sim_time true
>```

**En modo cámara física**, utiliza `use_sim_time=false` (valor predeterminado); si cambias de modo sin recrear los nodos, restablece ese parámetro a `false` en los cinco nodos. No actives el tiempo simulado sin una fuente de `/clock`.

Con los nodos en estado `unconfigured`, configúralos y, si la acción devuelve `success: true`, actívalos. Si ya están en `inactive`, ejecuta solo la activación; si ya están activos, no repitas las transiciones:


```bash
ros2 action send_goal /lifecycle_control hand_hri_mediapipe_msgs/action/LifecycleControl "{target_state: configure}" --feedback
ros2 action send_goal /lifecycle_control hand_hri_mediapipe_msgs/action/LifecycleControl "{target_state: activate}" --feedback
ros2 lifecycle get /hand_mediapipe_node
```

Tras configurar, el estado esperado es `inactive [2]`; tras activar, `active [3]`. Para comprobar el procesamiento:

```bash
ros2 topic echo /stats/hand_pipeline --once
```

No es necesario lanzar otra vez los nodos individuales de la sección **Nodos de ROS2**: el Compose ya los arranca mediante el launch lifecycle.

## 4. Cambiar de modo y comprobar el estado

Para pasar de cámara a rosbag, detén primero el driver RealSense y ejecuta el comando de arranque **sin cámara**. Para pasar a cámara, detén la reproducción del bag, prepara el USB y ejecuta el comando **con cámara**.

Cambiar los montajes puede recrear `operator_tracking`: los procesos iniciados con `docker exec` terminan y hay que volver a lanzar la fuente de datos y repetir la configuración y activación lifecycle. **No requiere reconstruir la imagen.**

Consulta el estado y los logs con:

```bash
docker compose -f docker-compose.yml ps operator_tracking
docker compose -f docker-compose.yml logs --tail=80 operator_tracking
```

## 5. Red, RViz y servicios FIWARE

Se utiliza **Fast DDS** (`rmw_fastrtps_cpp`). Mantén el mismo `ROS_DOMAIN_ID` en los procesos que deban comunicarse y revisa `FASTDDS_BUILTIN_TRANSPORTS` en el Compose: `SHM` utiliza únicamente memoria compartida local; `DEFAULT` habilita los transportes predeterminados, incluido UDP, para comunicar también con otros equipos si la red lo permite. La configuración se aplica al crear los procesos, no solo al abrir una nueva terminal.

Este Compose utiliza `network_mode: host`. En Docker Desktop activa **Settings → Resources → Network → Enable host networking** y aplica los cambios; véase la [documentación de Docker](https://docs.docker.com/engine/network/drivers/host/).

Para visualizar un rosbag reproducido con `--clock`, arranca RViz con `rviz2 --ros-args -p use_sim_time:=true`. En modo cámara física utiliza el tiempo del sistema (`use_sim_time=false`). En RobotModel selecciona `/robot_description` como topic de descripción. Si el modelo no aparece al abrir RViz después de que se haya publicado su mensaje único, revisa el QoS transient-local de ese topic y del consumidor.

En Windows, ejecutar `rviz2` dentro del contenedor requiere configurar además el acceso gráfico a **WSLg** (variables de pantalla y montaje del socket). El archivo USB solo añade acceso al dispositivo; no habilita la interfaz gráfica. Revisa también el QoS de los displays: en las pruebas realizadas, recibir las imágenes con **Reliable** recuperó la frecuencia esperada.

Los comandos anteriores arrancan solo `operator_tracking`. El Compose principal también define Orion-LD, MongoDB, TimescaleDB, Adminer y Grafana. Antes de arrancar todos los servicios debe existir el mapping `fiware/out/dds-config.json` como archivo válido; consulta [la guía FIWARE](README_parteFIWARE.md).

Una vez preparado el mapping y sin otro despliegue FIWARE duplicado, puedes arrancar todos los servicios:

```bash
# Rellenar con resultado
```
Lanza tus nodos de ROS en Jazzy o en un contenedor Jazzy usando la red del Host.

>[!NOTE] 
> El `docker-compose.yml` que proporcionamos ya lleva incluido el nodo de ROS2 que vamos a utilizar en el Workshop, pero si alguien está interesado en una versión general que puedan usar para sus desarrollos, proporcionamos también `docker-compose-orion.yml`.
>```bash
>docker compose -f docker-compose-orion.yml up -d
>```


El compose levanta las siguientes aplicaciones:
- Adminer: Visualizador de la tabla de datos. Puerto `8082`, `Username: orion` & `Password: orion` & `Base de datos: orion`.
- Grafana: Dashboard que usaremos al final del workshop para visualizar los datos. Puerto `4430`, `Username: admin` & `Password: admin`

## Nodos de ROS2



Si queréis una explicación más detallada de los nodos podréis consultarlo [aquí](/arise-vulcanexus-enviroment/docs/Nodos.md)

[Nodos de ciclo ](/arise-vulcanexus-enviroment/docs/Configurar-lifecycle.md)

- Nodo cámara
```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  enable_gyro:=false \
  enable_accel:=false 
```

>[!NOTE] USB2.1
> En caso de que la cámara se conecte aun USB2.1 recomendamos lanzarlo de la siguiente manera
>```bash
>ros2 launch realsense2_camera rs_launch.py \
>     camera_name:=cameraD455 \
>     device_type:=d455 \
>     enable_color:=true \
>     enable_depth:=true \
>     enable_gyro:=false \
>     enable_accel:=false \
>     align_depth.enable:=true \
>     pointcloud.enable:=true \
>     pointcloud.allow_no_texture_points:=false \
>     rgb_camera.color_profile:=640x480x15 \
>     depth_module.depth_profile:=640x480x15 \
>     decimation_filter.enable:=false \
>     pointcloud.ordered_pc:=true \
>     clip_distance:=2.0 \
>     publish_tf:=true
>```
- Mediapipe hand node
```bash
ros2 run hand_hri_mediapipe hand_mediapipe_node
```
- Hand landmarks 3d
```
ros2 run hand_hri_mediapipe hand_landmarks_3d_node
```
- Plane hand palm
```bash
ros2 run hand_hri_mediapipe hand_palm_plane_node
```

- Plano de camara a base Robot - Mano
```
ros2 run hand_hri_mediapipe hand_cam_to_base_node
```
- Launch 
```bash
ros2 launch hand_hri_mediapipe hand_pipeline.launch.py camera_name:=cameraD435f
```


<table>
  <tr>
    <td width="30%">
      <img src="docs/assets/CARTIFactory_robot_transparent.png" width="200" alt="Logo">
    </td>
    <td width="70%">
      <h3>CARTIFactory</h3>
      <p>
        Si estáis interesados en conocer más no olvidéis visitar la documentación de <a href="https://fundacion-cartif.github.io/CARTIFactory/es">CARTIFactory</a>!
        <br>
        Podéis solicitar una prueba experimental contactando con nosotros a través del siguiente <a href="mailto:marlod@cartif.es?subject=Solicitud%20Prueba%20CARTIFactory ">correo</a>.
      </p>
    </td>
  </tr>
</table>





>[!tip] Proyecto ARISE
><p align="center">
>  <!-- ---------- ARISE logo ---------- -->
>  <!-- Light mode -->
>  <img src="docs/assets/ARISE_logo-light_mode.png#gh-light-mode-only" alt="ARISE logo for light mode" height="60"/>
>  <!-- Dark mode -->
>  <img src="docs/assets/ARISE_logo-dark_mode.png#gh-dark-mode-only" alt="ARISE logo for dark mode" height="60"/>
></p>
>
> El proyecto [ARISE](https://arise-middleware.eu/) tiene como objetivo facilitar el acceso a tecnologías de interacción humano-robot (HRI) en entornos industriales, haciéndolas más accesibles y económicamente viables, especialmente en los sectores sanitario, intralogístico y manufacturero.
> 
>Estos módulos buscan integrar [FIWARE Orion Context Broker](https://fiware-orion.readthedocs.io/en/master/) y [eProsima Vulcanexus](https://vulcanexus.org/) para facilitar el desarrollo de aplicaciones robóticas e industriales capaces de adaptarse a su contexto. Además, incorporan [ROS4HRI](https://ros4hri.github.io/), un estándar abierto basado en ROS que proporciona un conjunto de paquetes diseñados para simplificar el desarrollo de capacidades de interacción humano-robot (HRI) en sistemas robóticos.
