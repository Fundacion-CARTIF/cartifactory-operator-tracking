# Descripción detallada de los nodos

A continuación proporcionamos una descripción más detallada de cada uno de los nodos de ROS2 que se proporcionan dentro del espacio de trabajo.

## Detección de manos en 2D

El nodo `hand_mediapipe_node` es el punto de entrada de la cadena de procesamiento de manos. Recibe imágenes RGB, realiza la inferencia de los puntos de referencia de las manos, identifica las manos izquierda y derecha y publica sus puntos de referencia 2D normalizados. También permite seleccionar durante la ejecución qué manos seguir (izquierda, derecha o ambas) mediante la interfaz de acciones del orquestador.

### Entradas

- Imágenes RGB de la cámara.

### Salidas

- Identificadores de las manos seguidas.
- Puntos de referencia 2D de las manos.
- Regiones de interés (ROI) 2D de las manos.
- Imagen de visualización para depuración.

### Interfaces

Este nodo dispone de las siguientes interfaces:

| Interfaz | Topic | Descripción |
| --- | --- | --- |
| Publicador | `/humans/hands/tracked` | Lista de identificadores de las manos que se están siguiendo. |
| Publicador | `/humans/hands/<id>/landmarks2d` | Puntos de referencia 2D normalizados de cada mano detectada. |
| Publicador | `/humans/hands/<id>/roi2d` | Cuadro delimitador 2D normalizado de cada mano detectada. |
| Publicador | `/humans/hands/debug_image` | Imagen de depuración con los puntos de referencia representados. |
| Suscriptor | `image_topic` | Topic de imágenes RGB configurado mediante el parámetro `image_topic`. Su valor predeterminado es `/camera/<camera_name>/color/image_raw`. |

### Selección de manos durante la ejecución

El nodo de detección permite seleccionar durante la ejecución las manos que se desean seguir: `left` (izquierda), `right` (derecha) o `both` (ambas).

La selección puede modificarse mientras el nodo está activo mediante la acción `HandChange` expuesta por el orquestador. No es necesario realizar ninguna transición de ciclo de vida ni reiniciar el nodo.

## Nodo de puntos de referencia 3D de las manos

El nodo `hand_landmarks_3d_node` sincroniza las detecciones 2D de las manos con las imágenes de profundidad y los parámetros intrínsecos de la cámara para estimar la posición 3D de cada punto de referencia de MediaPipe en el sistema de referencia de la cámara.

### Entradas

- Puntos de referencia 2D de las manos.
- Imagen de profundidad.
- Mensaje `CameraInfo` proveniente de la cámara.

### Salidas

- Puntos de referencia 3D de las manos.
- Marcadores de RViz.
- Nubes de puntos (opcionales).

### Interfaces

| Interfaz | Topic | Descripción |
| --- | --- | --- |
| Suscriptor | `/humans/hands/tracked` | Lista de identificadores de las manos que se están siguiendo. |
| Suscriptor | `/humans/hands/<id>/landmarks2d` | Puntos de referencia 2D normalizados de cada mano detectada. |
| Suscriptor | `image_topic` | Topic de imágenes RGB configurado mediante el parámetro `image_topic`. Su valor predeterminado es `/camera/<camera_name>/color/image_raw`. |
| Suscriptor | `camera_info_topic` | Parámetros intrínsecos de la cámara utilizados para la proyección 3D. Su valor predeterminado es `/camera/<camera_name>/aligned_depth_to_color/camera_info`. |
| Publicador | `/humans/hands/<id>/landmarks3d` | Puntos de referencia 3D de la mano expresados en el sistema de referencia de la cámara. |
| Publicador | `/humans/hands/<id>/landmarks3d_marker` | Visualización de los puntos de referencia 3D en RViz. |
| Publicador | `/humans/hands/<id>/skeleton3d_marker` | Visualización del esqueleto de la mano en RViz. |
| Publicador | `/humans/hands/<id>/landmarks3d_cloud` | Representación opcional de los puntos de referencia de la mano como `PointCloud2`. |

> [!IMPORTANT] Adaptación dinámica
> Las pipeline de procesamiento se crean dinámicamente para cada mano que se esté siguiendo. Cuando aparece un nuevo identificador de mano en `/humans/hands/tracked`, el nodo crea automáticamente los suscriptores, sincronizadores y publicadores necesarios. Los recursos se liberan cuando se deja de seguir esa mano.

### Reconstrucción 3D

Para cada punto de referencia 2D válido, el nodo:

1. Convierte las coordenadas normalizadas de la imagen en coordenadas de píxel.
2. Obtiene el valor de profundidad correspondiente.
3. Utiliza los parámetros intrínsecos de la cámara para realizar la retroproyección del píxel a un punto 3D.
4. Publica el punto de referencia resultante en el sistema de referencia de la cámara.

Las medidas de profundidad no válidas **se descartan automáticamente** y se marcan como no válidas en el mensaje de salida.

## Nodo del plano de la palma de la mano

El nodo `hand_palm_plane_node` ajusta un plano utilizando un subconjunto configurable de puntos de referencia de la mano y proporciona una estimación estable de la orientación de la palma. Esta información puede utilizarse para el análisis de gestos, la interacción humano-robot, la estimación de la dirección de señalamiento o la inferencia de la intención de agarre.

### Entradas

- Puntos de referencia 3D de las manos.

### Salidas

- Estimación del plano de la palma.
- Marcador del centroide de la palma.
- Marcador del vector normal de la palma.
- Malla de la superficie de la palma (opcional).

### Interfaces

| Interfaz | Topic | Descripción |
| --- | --- | --- |
| Suscriptor | `/humans/hands/tracked` | Lista de identificadores de las manos que se están siguiendo. |
| Suscriptor | `/humans/hands/<id>/landmarks3d` | Puntos de referencia 3D de la mano expresados en el sistema de referencia de la cámara. |
| Publicador | `/humans/hands/<id>/palm_plane_cam` | Plano estimado de la palma expresado en el sistema de referencia de la cámara. |
| Publicador | `/humans/hands/<id>/palm_plane_centroid_marker` | Marcador de RViz que representa el centroide de la palma. |
| Publicador | `/humans/hands/<id>/palm_plane_normal_marker` | Marcador de RViz que representa el vector normal de la palma. |
| Publicador* | `/humans/hands/<id>/palm_plane_mesh_marker` | Visualización opcional en RViz de la malla de la superficie estimada de la palma. |

### Estimación del plano

El plano de la palma se estima a partir de un subconjunto configurable de puntos de referencia mediante un ajuste de plano por mínimos cuadrados basado en la descomposición en valores singulares (SVD).

El vector normal resultante puede estabilizarse mediante:

- Orientación inicial hacia la cámara.
- Mantenimiento de la coherencia del signo.
- Limitación del incremento angular.
- Filtrado mediante media móvil exponencial.

### Generación de la malla

Cuando esta función está habilitada, el nodo proyecta los puntos de referencia válidos sobre el plano estimado, calcula su envolvente convexa 2D, triangula el polígono y publica un marcador de RViz de tipo `TRIANGLE_LIST` que representa la superficie de la palma.

## Transformación de las manos del sistema de referencia de la cámara al del robot

El nodo `hand_cam_to_base_node` es útil cuando la cámara está montada sobre un robot, y queremos mover los puntos de un sistema de referencia a otro. Aunque no se vaya a poder demostrar en el workshop, consideramos que puede ser de utilidad para alguna aplicación.

Utilizando la pose actual del TCP del robot y una transformación calibrada entre la cámara y el TCP, el nodo calcula los puntos de referencia de las manos y los planos de las palmas expresados en coordenadas del robot. Esto permite que las aplicaciones robóticas posteriores trabajen directamente con el espacio de trabajo del robot.

### Entradas

- Puntos de referencia de las manos en el sistema de referencia de la cámara.
- Plano de la palma en el sistema de referencia de la cámara.
- Pose del TCP.

### Salidas

- Puntos de referencia de las manos en el sistema de referencia de la base del robot.
- Plano de la palma en el sistema de referencia de la base del robot.
- Marcadores opcionales de RViz.

### Interfaces

Este nodo dispone de las siguientes interfaces:

| Interfaz | Topic | Descripción |
| --- | --- | --- |
| Suscriptor | `/humans/hands/tracked` | Lista de identificadores de las manos que se están siguiendo. |
| Suscriptor | `/humans/hands/<id>/landmarks3d` | Puntos de referencia 3D de la mano expresados en el sistema de referencia de la cámara. |
| Suscriptor | `/humans/hands/<id>/palm_plane_cam` | Estimación del plano de la palma expresada en el sistema de referencia de la cámara. |
| Suscriptor | `tcp_pose_topic` | Pose actual del TCP del robot utilizada para calcular la transformación de la cámara a la base. Su valor predeterminado es `/ur/tcp_pose`. |
| Publicador | `/humans/hands/<id>/landmarks3d_base` | Puntos de referencia 3D de la mano transformados al sistema de referencia de la base del robot. |
| Publicador | `/humans/hands/<id>/palm_plane_base` | Plano de la palma transformado al sistema de referencia de la base del robot. |
| Publicador* | `/humans/hands/<id>/centroid_marker` | Marcador opcional de RViz que muestra el centroide de la palma en el sistema de referencia de la base del robot. |
| Publicador* | `/humans/hands/<id>/normal_marker` | Marcador opcional de RViz que muestra el vector normal de la palma en el sistema de referencia de la base del robot. |


### Transformación de coordenadas

El nodo combina:

- La pose actual del TCP del robot.
- La transformación calibrada de la cámara al TCP (`T_cam_tcp`).
- Los puntos de referencia de la mano y el plano de la palma estimados en el sistema de referencia de la cámara.

Con estos datos, calcula la representación correspondiente de la mano en el sistema de referencia de la base del robot. Esto permite que las aplicaciones robóticas posteriores utilicen directamente las posiciones de las manos, los centroides de las palmas y sus vectores normales, sin necesidad de realizar transformaciones de coordenadas adicionales.



# Puntos de referencia

## Estadísticas de la cadena de procesamiento

El módulo incluye una interfaz de monitorización y diagnóstico (se utiliza la interfaz personalizada `custom_interfaces/PipelineStats`) que recopila información de ejecución de toda la cadena de seguimiento de manos.

Esta información está destinada a enviarse a sistemas externos de monitorización, como FIWARE Orion Context Broker, y puede utilizarse para la supervisión del sistema, la evaluación del rendimiento y el diagnóstico del despliegue.

| Campo | Tipo | Descripción |
| --- | --- | --- |
| `---` | `---` | --- |

> [!NOTE]
> La interfaz de estadísticas de la cadena de procesamiento se encuentra actualmente en desarrollo activo y puede evolucionar en futuras versiones. A medida que madure el módulo reutilizable, se incorporarán métricas de monitorización y campos de diagnóstico adicionales.

## Arquitectura de ciclo de vida

Todos los nodos de procesamiento se implementan como nodos de ciclo de vida de ROS 2.

Al iniciarse, todos los nodos permanecen en el estado `unconfigured`. El módulo incluye un nodo `orchestrator` que expone acciones de ROS 2 para gestionar el ciclo de vida de toda la cadena de procesamiento.

Transiciones de ciclo de vida admitidas:

- `configure`
- `activate`
- `deactivate`
- `cleanup`
- `shutdown`

Esta arquitectura permite que las aplicaciones externas inicien, detengan y reconfiguren toda la cadena de seguimiento de manos sin reiniciar el sistema.

## Acción de ciclo de vida

Todos los nodos comienzan en el estado `unconfigured` al lanzar el módulo. Para unificar la inicialización del sistema, se ha creado un `orchestrator` que aloja dos acciones. La primera permite cambiar el estado de todos los nodos en el orden correcto.

### Objetivo

| Campo | Tipo | Descripción |
| --- | --- | --- |
| `target_state` | `string` | Estado al que se desea cambiar el sistema. |

### Resultado

| Campo | Tipo | Descripción |
| --- | --- | --- |
| `success` | `bool` | Indica si la ejecución de la acción ha finalizado correctamente. |
| `result_msg` | `string` | Mensaje final resultante de la ejecución de la acción. |

### Información de progreso

| Campo | Tipo | Descripción |
| --- | --- | --- |
| `feedback` | `string` | Mensaje informativo que describe el estado actual de la ejecución de la acción. |

## Acción de cambio de mano

La segunda acción del `orchestrator` permite cambiar qué mano se va a seguir. Las opciones son **`left`**, **`right`** o **`both`**.

### Objetivo

| Campo | Tipo | Descripción |
| --- | --- | --- |
| `hand` | `string` | Mano que se desea seguir. Opciones: `left`, `right`, `both`. |

### Resultado

| Campo | Tipo | Descripción |
| --- | --- | --- |
| `success` | `bool` | Indica si la ejecución de la acción ha finalizado correctamente. |
| `result_msg` | `string` | Mensaje final resultante de la ejecución de la acción. |

### Información de progreso

| Campo | Tipo | Descripción |
| --- | --- | --- |
| `feedback` | `string` | Mensaje informativo que describe el estado actual de la ejecución de la acción. |

# Ejecución del módulo

El archivo de lanzamiento inicia toda la cadena de seguimiento de manos:

```bash
ros2 launch hand_hri_mediapipe hand_pipeline_lifecycle.launch.py
```

## Argumentos de lanzamiento

| Argumento | Valor predeterminado | Tipo | Descripción |
| --- | --- | --- | --- |
| `camera_name` | `cameraD455` | `string` | Espacio de nombres de la cámara utilizado para construir los topics de color, profundidad e información de la cámara. |
| `T_tcp_cam` | `identity/calibrated default` | Array de `float` `[16]` | Matriz de transformación homogénea del sistema de referencia del TCP del robot al de la cámara, utilizada para transformar los puntos de referencia de las manos y los planos de las palmas al sistema de referencia de la base del robot. |

> [!IMPORTANT]
> La matriz `T_tcp_cam` debe adaptarse a la calibración real entre la cámara y el robot de la instalación. Una calibración incorrecta producirá posiciones erróneas de los puntos de referencia de las manos y de los planos de las palmas en el sistema de referencia de la base del robot.

---

> [!NOTE] Interfaces personalizadas
> Los nodos anteriormente descritos utilizan definiciones de mensajes y acciones personalizadas, si se quieren ver con más profundidad proporcionamos una [descripción más detallada](/arise-vulcanexus-enviroment/docs/Interfaces.md).
