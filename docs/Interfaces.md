# Interfaces personalizadas

Se incluye un paquete de interfaces personalizadas para gestionar los mensajes personalizados que contienen la información de la cadena de procesamiento y el objetivo.

## Mensaje de seguimiento de manos

Definición del mensaje que representa una mano seguida mediante los puntos de referencia de MediaPipe:

![Puntos de referencia de la mano](/arise-vulcanexus-enviroment/docs/assets/)

| Campo | Tipo | Descripción |
| --- | --- | --- |
| `id` | `string` | Identificador único de la mano detectada o de la persona seguida. |
| `handedness` | `uint8` | Indica si la mano detectada es la izquierda o la derecha. |
| `landmarks` | `hri_msgs/NormalizedPointOfInterest2D[21]` | Array que contiene los 21 puntos de referencia de la mano de MediaPipe en coordenadas normalizadas de la imagen. |
| `confidence` | `float32` | Valor de confianza de la detección y el seguimiento de la mano. |

> [!NOTE]
> **Convención para identificar la mano**
>
> | Constante | Valor | Descripción |
> | --- | --- | --- |
> | `HANDEDNESS_UNKNOWN` | `0` | Mano no identificada. |
> | `HANDEDNESS_LEFT` | `1` | Mano izquierda detectada. |
> | `HANDEDNESS_RIGHT` | `2` | Mano derecha detectada. |

## Mensaje de seguimiento de manos en 3D

Definición del mensaje que representa una mano seguida en el espacio 3D mediante los puntos de referencia de MediaPipe.

| Campo | Tipo | Descripción |
| --- | --- | --- |
| `id` | `string` | Identificador único de la mano detectada o de la persona seguida. |
| `handedness` | `uint8` | Indica si la mano detectada es la izquierda o la derecha. |
| `confidence` | `float32` | Valor de confianza del clasificador de lateralidad de la mano. Si no está disponible, se utiliza `NaN` o `0`. |
| `landmarks` | `geometry_msgs/Point[21]` | Array que contiene los 21 puntos de referencia de la mano de MediaPipe en coordenadas normalizadas de la imagen. |
| `valid` | `bool[21]` | Indica si cada punto de referencia es válido o está siendo seguido. |

> [!NOTE]
> Las coordenadas de los puntos de referencia se expresan en el espacio 3D mediante `geometry_msgs/Point`. El array `valid` permite comprobar si cada punto de referencia se ha estimado correctamente. `valid[i] == false` significa que el punto de referencia `i` no debe considerarse fiable.