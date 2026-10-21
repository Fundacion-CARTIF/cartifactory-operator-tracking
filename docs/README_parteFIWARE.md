# Mapping Tool DDS -> NGSI-LD -> Arrancar ecosistema FIWARE

Este proyecto describe el flujo **DDS/ROS 2 → NGSI-LD → FIWARE** en **dos etapas independientes**. Primero, el **Mapping Tool** descubre topics, servicios y acciones DDS/ROS 2 y permite configurar su representación como entidades NGSI-LD. Después, **Orion-LD** utiliza el JSON generado para integrar los datos DDS con el Context Broker, apoyado por MongoDB, TimescaleDB, Adminer y Grafana.

Se incluyen dos archivos Docker Compose: `docker-compose-mapping.yml` (descubrimiento y edición web) y `docker-compose-fiware.yml` (broker, bases de datos y herramientas de consulta). **Generar el mapping no pone en marcha el broker**: hay que lanzar explícitamente la segunda etapa una vez disponible `fiware/out/dds-config.json`.

## 1. Objetivo y alcance

El Mapping Tool permite:

- **Descubrir** topics, servicios y acciones DDS/ROS 2 sin introducirlos manualmente.
- **Configurar** las correspondencias entre endpoints DDS y entidades, tipos y atributos NGSI-LD.
- **Revisar y modificar** los mappings desde una interfaz web.
- **Generar y guardar** `dds-config.json` y `dds-context.jsonld` para una integración posterior con Orion-LD.
- **Arrancar la integración FIWARE** una vez generado el mapping: Orion-LD, MongoDB, TimescaleDB, Adminer y Grafana.

Después de generar los archivos, el segundo Compose permite **poner en marcha el Context Broker Orion-LD**, disponer de almacenamiento de estado e histórico y consultar los datos mediante Adminer o Grafana. La publicación ROS 2 sigue siendo una actividad externa a ambos Compose.

## 2. Fundamentos teóricos

### ROS 2 y DDS

En **ROS 2**, los programas se organizan en nodos que intercambian información. Un **topic** transporta mensajes publicados por uno o varios nodos a quienes se suscriben. Un **servicio** permite realizar peticiones y recibir respuestas; una **acción** representa tareas que pueden durar más tiempo e informar de su progreso.

**DDS (Data Distribution Service)** es una tecnología de comunicación utilizada por ROS 2. Entre otras funciones, permite que los participantes descubran qué topics y tipos de datos existen en un dominio DDS. El **dominio** identifica una red lógica de comunicación; en este ejemplo trabajamos con el dominio **0**.

### DDS-Enabler Discovery

**DDS-Enabler** es un proyecto de eProsima que permite integrar sistemas DDS con aplicaciones externas. Para este despliegue se compila su rama `example/discovery-ws`, que incluye un ejemplo de **discovery mediante WebSocket**.

Este servicio observa los elementos DDS visibles en su dominio y expone la información al mapper en:

`ws://localhost:8081`

> Ese WebSocket proporciona **metadatos de discovery**. No se debe confundir con el flujo de datos de sensores hacia una base de datos.

### FIWARE, NGSI-LD y el mapper

**NGSI-LD** es un modelo y una API para representar información de contexto como **entidades**, **propiedades**, **relaciones** e identificadores **IRI**. En una integración de robótica con FIWARE se puede representar, por ejemplo, el topic ROS 2 `/positions` como un atributo `positions` de una entidad NGSI-LD.

El **DDS-NGSI-LD Mapper** ayuda a realizar esa correspondencia. Para cada elemento descubierto permite decidir si se mapea, se omite o se bloquea, y editar valores como:

- `entityType`: tipo lógico de entidad, por ejemplo `Positions`.
- `entityId`: identificador de entidad, por ejemplo `urn:ngsi-ld:Positions:positions`.
- `attribute`: propiedad asociada, por ejemplo `positions`.

El mapper genera **configuración**, no convierte por sí solo cada publicación ROS 2 en un registro histórico.

### Orion-LD, estado actual e histórico temporal

**Orion-LD** es un **Context Broker NGSI-LD**. En la integración DDS descrita por ARISE, se inicia con `-wip dds` y lee su configuración desde `/root/.orionld` dentro del contenedor; aquí ese archivo corresponde al `dds-config.json` producido por el Mapping Tool. Orion-LD participa en el mismo dominio DDS y utiliza las reglas del mapping para asociar la información con entidades NGSI-LD. **No depende del WebSocket de discovery para procesar los datos:** el WebSocket es la herramienta previa de configuración.

**MongoDB** almacena el estado de las entidades gestionado por Orion-LD. **TimescaleDB** (PostgreSQL con extensión de series temporales) se usa para su representación **temporal/histórica** mediante `-troe` (*Temporal Representation of Entities*). Así, la información más reciente y su evolución histórica responden a necesidades distintas. La existencia de registros depende de que Orion-LD se inicie correctamente y reciba datos compatibles.

### Consultas y visualización

**Adminer** permite consultar la base PostgreSQL desde el navegador. **Grafana** permite explorar series temporales, construir consultas SQL y crear paneles; en este proyecto se aprovisiona automáticamente su conexión PostgreSQL y una fuente **Infinity** opcional para fuentes HTTP/JSON. Ni Adminer ni Grafana descubren topics ROS 2 ni realizan el mapping.


## 3. Arquitectura del sistema

Hay dos fases consecutivas, que pueden convivir en el mismo equipo y disponen de **Compose diferentes**:

```text
FASE 1 · DEFINICIÓN DEL MAPPING
Nodo ROS 2 / Vulcanexus (DDS, dominio 0)
      │
      ▼
DDS-Enabler Discovery (WebSocket, localhost:8081)
      │ información sobre topics, servicios y acciones
      ▼
Mapping Tool Web (HTTP, localhost:3001)
      │ editar entityType / entityId / attribute
      ▼
fiware/out/dds-config.json + dds-context.jsonld
      │
      │ configurar Orion-LD (archivo .orionld)
      ▼
FASE 2 · EJECUCIÓN DE FIWARE
Nodo ROS 2 ── DDS (dominio 0) ──► Orion-LD (API :1026)
                                       │
                            ┌──────────┴──────────┐
                            ▼                     ▼
                    MongoDB (estado)      TimescaleDB (histórico)
                                                  │
                                          ┌───────┴───────┐
                                          ▼               ▼
                                    Adminer :8082    Grafana :4430
```

En el Compose del mapper, `discovery` y `mapper-web` utilizan `network_mode: host` en Linux. En la segunda etapa, **MongoDB y Orion-LD** también comparten la red del host, mientras que **TimescaleDB, Adminer y Grafana** utilizan la red interna de su Compose. Por eso `timescale-db:5432` es válido dentro de Grafana, mientras que Orion-LD, en la red del host, se conecta mediante `localhost:5433` (puerto publicado).

| Servicio | Dirección en este ejemplo | Uso |
| --- | --- | --- |
| DDS-Enabler Discovery | `ws://localhost:8081` | Discovery para diseñar los mappings. |
| Mapping Tool Web | `http://localhost:3001` | Editor y generador de JSON. |
| Orion-LD | `http://localhost:1026` | API NGSI-LD y participación DDS. |
| TimescaleDB | `127.0.0.1:5433` en el host / `timescale-db:5432` dentro de Compose | Histórico temporal. |
| Adminer | `http://localhost:8082` | Consultas SQL por navegador. |
| Grafana | `http://localhost:4430` | Paneles y consultas de datos. |

**Por qué se han elegido estos puertos:** los puertos del ejemplo permiten ejecutar las dos etapas a la vez y reducen posibles colisiones con servicios habituales. El discovery usa `8081`, por lo que Adminer se publica en **8082**. TimescaleDB se publica en **5433** (solo en `127.0.0.1`) para evitar ocupar el puerto PostgreSQL habitual `5432` del equipo; dentro de Docker sigue escuchando en `5432`. Antes de arrancar, comprueba que los puertos están libres en **tu ordenador**. Si necesitas cambiarlos, actualiza también las direcciones o variables de los servicios que se conectan a ellos.

## 4. Archivos del proyecto

La estructura esperada, **relativa a ambos archivos Compose**, es:

```text
arise-vulcanexus-enviroment/
├── docker-compose-mapping.yml        # Fase 1: discovery + Mapping Tool
├── docker-compose-fiware.yml         # Fase 2: Orion-LD + bases de datos + visualización
├── start-mapping.sh                  # Arranque fácil de la fase 1
├── README.md
├── docsFIWARE/
│   └── images/                      # Capturas ya incluidas en esta guía
│       ├── 01-vista-general.png
│       ├── 02-cargar-discovery.png
│       ├── 03-editar-mappings.png
│       └── 04-generar-archivos.png
└── fiware/
    ├── discovery/
    │   └── Dockerfile               # Compila DDS-Enabler desde GitHub
    ├── grafana_datasources/
    │   └── datasources.yaml         # Aprovisionamiento de PostgreSQL e Infinity
    └── out/                         # Archivos generados por el mapper
        ├── dds-config.json          # Aparece tras guardar el mapping
        └── dds-context.jsonld       # Aparece tras guardar el mapping
```

`fiware/out/` y sus JSON no tienen por qué existir al clonar: el script del mapper crea la carpeta y la web genera los archivos. **Antes de iniciar FIWARE debe existir `dds-config.json` como archivo**, no como directorio. Los volúmenes de las bases de datos y Grafana los gestiona Docker y no aparecen como carpetas de este repositorio.

El Compose construye:

- **`discovery`** desde `./fiware/discovery/Dockerfile`. Ese Dockerfile descarga de GitHub la rama `example/discovery-ws` de **eProsima/DDS-Enabler**, instala dependencias y compila con `colcon`.
- **`mapper-web`** desde el repositorio público [`dds-ngsild-mapper`](https://github.com/Engineering-Research-and-Development/dds-ngsild-mapper), utilizando su `Dockerfile` en la rama `main`.

La carpeta `./fiware/out` se monta como `/app/out` dentro del mapper. Los archivos que la web guarde permanecerán en el ordenador aunque se eliminen los contenedores.

### Configuración que se crea al arrancar

El Compose genera `/tmp/discovery-config.yml` **dentro del contenedor discovery**, con este contenido:

```yaml
dds:
  domain: 0
```

Después carga `/ws/install/setup.sh` (para encontrar las librerías compartidas de DDS-Enabler) y ejecuta `ddsenabler_example_discovery` con la ruta de configuración y el puerto `8081`. La ruta de configuración **no puede estar vacía**: de lo contrario el programa falla al crear el file watcher.

> La validez y semántica de la configuración dependen de la versión de la rama utilizada. Si discovery no llega al estado operativo, revisa sus logs y adapta ese YAML siguiendo la documentación de la versión compilada.

## 5. Requisitos previos

- Un equipo con **Linux nativo** o **Windows con WSL2** y Docker (preferiblemente **Docker Desktop** con integración WSL2), utilizando **contenedores Linux** y Docker Compose.
- El Compose utiliza `network_mode: host`. En **Docker Desktop 4.34 o posterior** hay que activar **Settings → Resources → Network → Enable host networking**. En Linux nativo, la red host está disponible en Docker Engine. En WSL2 con un Docker Engine instalado directamente en la distribución (sin Docker Desktop), revisa las particularidades de red de WSL2.
- Permisos para ejecutar Docker con tu usuario (no es necesario utilizar `sudo` si ya tienes acceso al daemon).
- Conexión a Internet durante la primera construcción: Docker descarga imágenes, paquetes y código fuente público.
- Puertos **8081** y **3001** libres para el mapper. Para FIWARE, además: **1026**, **8082**, **4430**, **5433** (loopback) y **27017** (loopback).
- Para obtener descubrimientos reales, debe existir algún publicador o servicio DDS/ROS 2 visible en el dominio **0**.

Comprobaciones rápidas:

```bash
docker --version
docker compose version
id -u
id -g
```

> Si un puerto ya está ocupado en tu equipo, comprueba qué lo utiliza antes de modificarlo. Para cambiar un puerto, modifica tanto el servicio que lo escucha como la URL o variable que lo utiliza. Las conexiones DDS/ROS 2 pueden requerir ajustes adicionales de red en WSL2, especialmente si los nodos están fuera del entorno Linux.

## 6. Arranque rápido (recomendado)

Desde la raíz del repositorio:

```bash
cd /ruta/a/arise-vulcanexus-enviroment
bash start-mapping.sh
```

El script proporcionado:

1. Se sitúa en la carpeta del proyecto.
2. Crea `fiware/out` si todavía no existe.
3. Detecta tu **UID y GID** con `id -u` e `id -g` y exporta `MAPPER_UID` y `MAPPER_GID` para Docker Compose.
4. Construye las imágenes necesarias y levanta los servicios en segundo plano.

Se utiliza porque tu Compose contiene:

```yaml
user: "${MAPPER_UID}:${MAPPER_GID}"
```

Con esto el mapper intenta escribir los JSON con **tu usuario Linux**, evitando archivos nuevos propiedad de `root`. **No corrige permisos de ficheros antiguos** que ya pertenezcan a otra persona.

Cuando termine:

```bash
bash start-mapping.sh status
```

**Primera instalación en un ordenador nuevo:** basta con `bash start-mapping.sh`. El script usa `docker compose up -d --build`, por lo que Docker descargará y construirá las imágenes necesarias automáticamente; **no hay que ejecutar `rebuild` antes**. La primera construcción puede tardar varios minutos; los siguientes arranques suelen ser más rápidos. El comando `bash start-mapping.sh rebuild` se reserva para forzar una reconstrucción completa sin caché. Cuando termine, ambos servicios deberían mostrar un estado `Up` o `running`.

### Alternativa sin el script

Si solo tienes el archivo Compose, puedes utilizar:

```bash
mkdir -p fiware/out
MAPPER_UID="$(id -u)" MAPPER_GID="$(id -g)" \
  docker compose -f docker-compose-mapping.yml up -d --build
```

Para posteriores arranques, **vuelve a proporcionar esas variables** o utiliza `start-mapping.sh`; no ejecutes un `docker compose up` sin ellas.

## 7. Abrir la interfaz web (Linux o WSL2)

Abre el navegador de tu **propio ordenador** y visita:

**`http://localhost:3001`**

- **Linux nativo:** abre esta dirección en el navegador de Linux.
- **Windows con WSL2:** arranca el proyecto desde la terminal Linux de WSL2 y abre la misma dirección en el navegador de **Windows**. En una configuración compatible, el servicio será accesible mediante `localhost`. Si no responde, comprueba que Docker Desktop tiene habilitada la integración con WSL2 y la opción **host networking**, así como el reenvío de puertos de WSL2.

Si la página no abre, comprueba primero que el mapper esté levantado y que responda en el entorno donde ejecutas Docker:

```bash
bash start-mapping.sh status
curl -I http://localhost:3001
```

En **Discovery source**, mantén configurado:

**`ws://localhost:8081`**

Esa dirección la usa el **backend del mapper** para conectarse a DDS-Enabler, no el navegador directamente. Con la configuración `network_mode: host` de ambos servicios, `localhost:8081` identifica el WebSocket del entorno donde se ejecutan los contenedores.

## 8. Cómo utilizar el Mapping Tool (guía visual)

Una vez que los contenedores estén en funcionamiento y hayas abierto la web, sigue este recorrido. Las capturas corresponden a una sesión de ejemplo; **los topics y servicios concretos pueden variar** según los nodos ROS 2 activos en cada equipo.

### Vista general de la aplicación

![Vista general del DDS a NGSI-LD Mapping Tool, con Discovery source, Global settings y Smart Data Models](docsFIWARE/images/01-vista-general.png)

*Figura 1. Pantalla principal del Mapping Tool.*

La interfaz está organizada en distintas secciones: **1. Discovery source**, **2. Global settings**, **3. Mapping**, **4. Smart Data Models** (opcional) y **5. Generate output**. El proceso consiste en **descubrir** los endpoints DDS, **revisar** qué entidades NGSI-LD representarán y **guardar** la configuración resultante.

### Paso 1. Cargar el descubrimiento DDS

![Detalle de Discovery source, con Backend URL, modo Interactive y botón Load discovery](docsFIWARE/images/02-cargar-discovery.png)

*Figura 2. Selección de la fuente de descubrimiento.*

1. En **Discovery source**, selecciona **Backend URL** e introduce `ws://localhost:8081`.
2. En **Mapping mode**, elige **Interactive** para revisar y modificar manualmente las sugerencias. El modo **Automatic** aplica el mapeo propuesto automáticamente, con menor intervención del usuario.
3. Mantén activado **Auto-blocklist log topics** si no quieres mapear mensajes de registro como `/rosout`.
4. Pulsa **Load discovery**. El backend del mapper contactará con DDS-Enabler y cargará los topics, servicios y acciones que descubra.

> **Importante sobre `localhost`:** tanto si abres la interfaz desde Linux como desde el navegador de Windows con WSL2, el WebSocket de discovery lo consulta el **backend del mapper**, no el navegador. Por eso la dirección `ws://localhost:8081` es la adecuada cuando el mapper y DDS-Enabler se ejecutan en el mismo entorno Docker con `network_mode: host`.
>
> Las pestañas **Upload file** y **Paste JSON** permiten suministrar datos de discovery desde un archivo o texto JSON. No son una función para abrir y editar directamente un `dds-config.json` ya generado.

Si no aparece el topic esperado, comprueba que existe un publicador DDS/ROS 2 activo en el dominio `0` y que el servicio discovery funciona (`docker logs --tail 50 ddsenabler-discovery`).

### Paso 2. Revisar los ajustes globales

En **Global settings**, visible en la **Figura 1**, encontrarás parámetros que afectan al resultado generado:

| Campo | Función | Valor de ejemplo |
| --- | --- | --- |
| **IRI base** | Prefijo utilizado para las IRIs de tipos y atributos NGSI-LD. | `https://my.org/dds/` (ejemplo) |
| **@context URI (-duc)** | URI de un contexto JSON-LD que puede utilizarse en la integración posterior. | Se deja sin completar si no se ha definido uno. |
| **DDS domain** | Dominio DDS que debe corresponder con el utilizado por ROS 2 y discovery. | `0` |
| **Types directory** | Ruta del directorio de tipos DDS dentro del entorno de ejecución. | `/opt/dds/types` |
| **Sync timeout (ms)** | Tiempo de espera de las operaciones de sincronización configuradas. | `5000` |
| **Output config** | Archivo de configuración que se generará. | `out/dds-config.json` |
| **Output @context** | Archivo JSON-LD que se generará. | `out/dds-context.jsonld` |

La base `https://my.org/dds/` es **solo ilustrativa**. En una integración real conviene utilizar una base IRI controlada por el proyecto para que los identificadores sean consistentes. El parámetro **Sync timeout** no es el timeout del WebSocket de discovery (`DDS_DISCOVERY_TIMEOUT_MS`, configurado en el Compose).

### Paso 3. Seleccionar y editar qué endpoints mapear

![Tabla de Mapping con los topics rt/positions, rt/rosout y los campos entityType, entityId y attribute](docsFIWARE/images/03-editar-mappings.png)

*Figura 3. Tabla de mapeo de topics y servicios detectados.*

Después de cargar el discovery aparece la tabla **3. Mapping**. Cada fila representa un elemento DDS descubierto. En la captura se ve `rt/positions`: es la **representación DDS** del topic ROS 2 `/positions`.

Para cada fila hay tres acciones:

- **Map**: incluir el endpoint en el mapeo NGSI-LD.
- **Skip**: omitir ese endpoint del mapeo.
- **Blocklist**: incorporarlo a los elementos excluidos explícitamente, útil para topics de sistema o ruido de diagnóstico.

En el ejemplo, localiza la fila **`rt/positions`**, elige **Map** y revisa sus tres campos:

| Campo | Valor de ejemplo | Significado |
| --- | --- | --- |
| `entityType` | `Positions` | Tipo de entidad NGSI-LD. |
| `entityId` | `urn:ngsi-ld:Positions:positions` | Identificador de la entidad. |
| `attribute` | `positions` | Atributo que representará la información del topic. |

Los campos vienen **pre-rellenados con sugerencias**, pero puedes editarlos. Los iconos de restauración recuperan el valor sugerido. Puedes desplegar **PAYLOAD** para consultar la información de tipos disponible y ayudarte a decidir el mapeo.

En el ejemplo, `/rosout` está en **Blocklist**; otros topics de sistema, como `/parameter_events`, también se pueden omitir si no son relevantes para la integración. Evita usar **Map all** sin revisar las filas: podrías añadir endpoints internos que no necesitas. Las acciones masivas **Map all**, **Skip all** y **Blocklist all** afectan a varias entradas a la vez.

**Servicios y acciones** aparecen en secciones propias cuando existen. Descubrir un servicio no significa que debas mapearlo: selecciona únicamente los endpoints pertinentes para la aplicación.

### Paso 4. Smart Data Models (opcional)

La sección **4. Smart Data Models**, visible en la Figura 1, es un espacio de exploración para consultar modelos de datos publicados y tomar como referencia sus tipos y atributos. Puedes buscar, por ejemplo, términos relacionados con robots o dispositivos, inspeccionar un modelo y, si resulta adecuado, aprovechar su vocabulario en el mapeo.

No es obligatorio seleccionar un Smart Data Model: es una funcionalidad opcional que ayuda a reutilizar vocabularios existentes.

### Paso 5. Generar y guardar los archivos

![Generate output mostrando el botón Generate preview, la opción Also save to disk y los dos JSON resultantes](docsFIWARE/images/04-generar-archivos.png)

*Figura 4. Vista previa y guardado de la configuración `dds-config.json` y del contexto `dds-context.jsonld`.*

1. Ve a **5. Generate output**.
2. Activa **Also save to disk (out/)** si quieres conservar los archivos en el servidor.
3. Haz clic en **Generate preview** y revisa los dos JSON mostrados.
4. Comprueba que aparece un mensaje de confirmación, como **`OK — saved to out/dds-config.json and out/dds-context.jsonld`**.
5. Si lo necesitas, utiliza **Copy** para copiar el contenido o **Download** para descargar cada JSON desde el navegador.

Gracias al volumen del Compose `./fiware/out:/app/out`, al seleccionar el guardado en disco los archivos deben quedar **en el equipo anfitrión**:

```text
fiware/out/dds-config.json
fiware/out/dds-context.jsonld
```

Puedes comprobarlo desde una terminal situada en la raíz del repositorio:

```bash
ls -lh fiware/out/dds-config.json fiware/out/dds-context.jsonld
```

**Comprueba tu resultado:** `dds-config.json` debe reflejar la decisión de mapear `/positions` y las reglas de exclusión que hayas seleccionado. `dds-context.jsonld` debe definir las IRIs necesarias para los nombres de entidad y atributos empleados. **Estos archivos todavía no envían datos a Orion-LD por sí mismos**.

---

## 9. ¿Qué archivos se generan y para qué sirven?

La web debe escribir en el host:

```text
./fiware/out/dds-config.json
./fiware/out/dds-context.jsonld
```

- **`dds-config.json`**: describe el dominio DDS, filtros y la correspondencia entre endpoints DDS y `entityId`, `entityType` y atributos NGSI-LD.
- **`dds-context.jsonld`**: contiene el **contexto JSON-LD** que permite convertir nombres cortos de propiedades y tipos en IRIs completos.

Comprueba que existen:

```bash
ls -lh fiware/out/
```

**Importante:** estos archivos se generan para que **Orion-LD con su integración DDS** pueda usarlos posteriormente. Tener los JSON **no implica** que los mensajes `/positions` ya estén llegando a Orion-LD o guardándose en una base de datos.

## 10. Etapa 2: arrancar FIWARE después de generar el mapping

La fase anterior prepara dos ficheros JSON. En esta segunda etapa, **Orion-LD utiliza `dds-config.json` para integrar los endpoints DDS con el Context Broker**. El despliegue se describe en `docker-compose-fiware.yml` y es independiente de `docker-compose-mapping.yml`.

### 10.1. Procedencia y relación con el tutorial de ARISE

La base del servicio **Orion-LD** y de su almacenamiento se ha adaptado del tutorial oficial de ARISE [**Calling ROS 2 Actions from FIWARE Context Broker**](https://arise-framework-documentation.readthedocs.io/en/latest/rst/tutorials/ros2_fiware_actions_app.html). Ese tutorial utiliza la imagen `fiware/orion-ld:1.13.0-PRE-1852`, habilita la integración DDS con `-wip dds`, habilita el histórico temporal con `-troe` y combina Orion-LD con MongoDB y TimescaleDB. Aquí **no reproducimos su ejemplo de la acción Fibonacci**: utilizamos el `dds-config.json` producido previamente por nuestro Mapping Tool para los endpoints que haya seleccionado el usuario.

| Componente / archivo | Procedencia | Adaptación en este proyecto |
| --- | --- | --- |
| **DDS-Enabler Discovery** | [eProsima/DDS-Enabler](https://github.com/eProsima/DDS-Enabler), rama `example/discovery-ws`. | Construido por `fiware/discovery/Dockerfile`; WebSocket en `8081`. |
| **DDS-NGSI-LD Mapper** | [Engineering-Research-and-Development/dds-ngsild-mapper](https://github.com/Engineering-Research-and-Development/dds-ngsild-mapper). | Interfaz web en `3001` que guarda la configuración en `fiware/out`. |
| **Orion-LD, MongoDB y TimescaleDB** | [Tutorial ARISE: ROS 2 Actions / FIWARE](https://arise-framework-documentation.readthedocs.io/en/latest/rst/tutorials/ros2_fiware_actions_app.html). | Utilizan el JSON del mapper; se adapta el puerto de TimescaleDB y se limita MongoDB a loopback. |
| **Adminer** | Extensión del Compose de este proyecto; imagen [Adminer (Docker Hub)](https://hub.docker.com/_/adminer). | Interfaz de administración de PostgreSQL en `8082`. |
| **Grafana** y `datasources.yaml` | Extensión de este proyecto; [aprovisionamiento oficial de Grafana](https://grafana.com/docs/grafana/latest/administration/provisioning/). | Fuente PostgreSQL y plugin [Infinity](https://grafana.com/docs/plugins/yesoreyeram-infinity-datasource/latest/installation/); interfaz en `4430`. |

### 10.2. Función de cada servicio del nuevo Compose

| Servicio | Imagen / función | Conexión y datos |
| --- | --- | --- |
| `timescale-db` | `timescale/timescaledb-postgis:1.7.5-pg12`; PostgreSQL 12 con TimescaleDB para histórico de entidades (*TRoE*). | Escucha en `5432` dentro del contenedor. Publica `127.0.0.1:5433` para Orion-LD y conserva la base en el volumen `timescale-db`. |
| `mongodb` | `mongo:7.0`; almacenamiento del estado actual de Orion-LD. | Comparte la red del host y escucha en `localhost:27017`. Su volumen es `mongo_data`. |
| `orion` | `fiware/orion-ld:1.13.0-PRE-1852`; Context Broker con integración DDS. | Lee `/root/.orionld`; accede a MongoDB por `localhost:27017` y a TimescaleDB por `localhost:5433`; su API escucha en `1026`. |
| `adminer` | `adminer:latest`; administrador web de bases de datos. | Accede a `timescale-db:5432` por la red Docker. Interfaz en `8082`. |
| `grafana` | `grafana/grafana:latest`; consultas, gráficos y dashboards. | Fuente PostgreSQL `timescale-db:5432` y plugin Infinity opcional; interfaz en `4430`. Sus ajustes persisten en `grafana_data`. |

**Parámetros importantes de Orion-LD:**

- `-dbhost localhost`: base MongoDB en la red compartida del host.
- `-wip dds`: habilita la integración DDS de Orion-LD; el broker lee `/root/.orionld` al arrancar.
- `-mongocOnly`: activa el driver MongoDB en C de Orion-LD, utilizado en este despliegue para trabajar con la versión moderna de MongoDB 7.0.
- `-troe`: habilita la representación temporal de las entidades, apoyada en PostgreSQL/TimescaleDB.
- `ORIONLD_TROE_HOST`, `ORIONLD_TROE_PORT`, `ORIONLD_TROE_USER`, `ORIONLD_TROE_PWD`: conexión de Orion-LD a TimescaleDB; **el puerto es 5433** porque Orion está en modo host y PostgreSQL se publica en ese puerto.
- `depends_on` y `healthcheck`: coordinan el arranque inicial. Que un contenedor esté levantado no garantiza por sí solo que el broker ya haya descubierto los endpoints ROS 2.
- `volumes`: persistencia de bases de datos y lectura de la configuración generada. El montaje de `dds-config.json` es **de solo lectura** y exige que el archivo ya exista, para evitar crear accidentalmente un directorio llamado `dds-config.json`.

**Diferencias deliberadas respecto al Compose original:** en el ejemplo proporcionado Adminer utilizaba `8081:8080`, pero **8081 ya pertenece a discovery**, así que se ha movido a `8082:8080`. Asimismo, TimescaleDB pasa de `5432:5432` a `127.0.0.1:5433:5432` para no ocupar el puerto PostgreSQL habitual `5432` del equipo; Orion-LD utiliza ahora `ORIONLD_TROE_PORT=5433`. MongoDB se limita a `127.0.0.1` porque Orion-LD está en la misma red host. Por último, se añade la instalación del plugin **Infinity** de Grafana y se asigna al Compose el nombre de proyecto `arise-fiware` para mantenerlo separado del despliegue del mapper.

### 10.3. Configuración automática de Grafana

El nuevo archivo `fiware/grafana_datasources/datasources.yaml`, que se monta en `/etc/grafana/provisioning/datasources/datasources.yaml`, registra dos fuentes:

1. **`orion`** (`type: postgres`): utiliza el host interno **`timescale-db:5432`**, usuario y base de datos `orion`, la contraseña de demostración `orion`, `postgresVersion: 1200` (PostgreSQL 12) y `timescaledb: true`. Es la fuente **predeterminada** para consultas y gráficos del histórico.
2. **`yesoreyeram-infinity-datasource`**: fuente opcional para leer datos de endpoints HTTP/JSON y otros formatos soportados por Infinity. El Compose incluye `GF_INSTALL_PLUGINS=yesoreyeram-infinity-datasource` para instalar el plugin; el YAML por sí solo no lo instala. Su lista de consultas globales parte vacía (`global_queries: []`).

El servicio Grafana persiste en `grafana_data` y expone el puerto **4430** del equipo anfitrión hacia el **3000 interno**. Se trata de **HTTP**, no HTTPS, aunque su puerto comience por `44`.

### 10.4. Puesta en marcha, paso a paso

**Paso 1 — Generar el mapping.** Arranca el editor si aún no lo has hecho, entra en **`http://localhost:3001`** desde Linux o desde el navegador de Windows con WSL2, pulsa **Load discovery**, configura los endpoints en **Mapping** y guarda con **Also save to disk (out/)**:

```bash
bash start-mapping.sh
ls -lh fiware/out/dds-config.json fiware/out/dds-context.jsonld
```

**Paso 2 — Comprobar el archivo que leerá Orion-LD.** La configuración real se monta desde **`./fiware/out/dds-config.json`** hasta **`/root/.orionld`** dentro de Orion-LD. Debe ser un **archivo JSON** generado por el mapper. Puedes comprobarlo con:

```bash
test -f fiware/out/dds-config.json && echo 'Mapping generado y disponible'
```

**Paso 3 — Comprobar puertos locales.** Antes de levantar FIWARE, revisa si están ocupados `1026`, `8082`, `4430`, `5433` o `27017`:

```bash
ss -ltn | grep -E ':(1026|8082|4430|5433|27017)\b' || true
```

**Paso 4 — Arrancar el segundo Compose.** Desde la raíz del repositorio:

```bash
docker compose -f docker-compose-fiware.yml up -d
```

La primera vez Docker descargará las imágenes de Orion-LD, MongoDB, TimescaleDB, Adminer y Grafana. **No hace falta utilizar `--build`**, ya que este Compose utiliza imágenes publicadas y no contiene bloques `build`. Para ver su estado y logs:

```bash
docker compose -f docker-compose-fiware.yml ps
docker compose -f docker-compose-fiware.yml logs -f orion
```

**Paso 5 — Verificar la API del broker.** Desde la terminal Linux (nativa o WSL2):

```bash
curl -s http://localhost:1026/version
```

El comando debe recibir una respuesta del servicio Orion-LD. Esto confirma que la API responde, **no demuestra todavía que esté recibiendo datos DDS**. Para eso, revisa los logs del broker y que exista un publicador DDS/ROS 2 compatible en el dominio `0`.

**Paso 6 — Parar FIWARE sin borrar las bases de datos:**

```bash
docker compose -f docker-compose-fiware.yml down
```

Los volúmenes `mongo_data`, `timescale-db` y `grafana_data` se conservan. **No ejecutes `down -v` salvo que realmente quieras borrar el estado y el histórico almacenados**.

### 10.5. Acceso local a Orion-LD, Adminer y Grafana

Abre las siguientes direcciones en el navegador de **Linux** o de **Windows si utilizas WSL2**:

| Componente | Dirección local | Función |
| --- | --- | --- |
| Orion-LD (API REST, sin interfaz gráfica) | `http://localhost:1026` | API NGSI-LD; puedes comprobarla con `curl`. |
| Adminer | `http://localhost:8082` | Administrador web de PostgreSQL. |
| Grafana | `http://localhost:4430` | Paneles y consultas de datos. |

Si utilizas WSL2 y alguna dirección no responde desde Windows, prueba primero el servicio con `curl` desde la terminal WSL2. A continuación, verifica la integración de Docker Desktop con WSL2, la red host y el reenvío de `localhost`. El acceso al navegador y el descubrimiento DDS son aspectos de red diferentes.

Para entrar en **Adminer**, utiliza el sistema **PostgreSQL**, servidor `timescale-db`, usuario `orion`, contraseña `orion` y base de datos `orion` (**solo credenciales de demostración**). El servidor no es `localhost` dentro de Adminer porque ese contenedor utiliza la red Docker interna.

En **Grafana**, entra en **Connections → Data sources** y verifica que exista la fuente `orion`; desde un panel puedes seleccionar esa fuente para crear consultas PostgreSQL. La fuente Infinity aparecerá si el plugin se ha instalado correctamente. Configura credenciales de administrador propias de Grafana antes de exponerlo a otras personas.

Si Orion-LD está registrando información temporal, desde el editor SQL de Adminer podrías consultar, **si existe la tabla y has generado ese mapping**:

```sql
SELECT *
FROM public.attributes
WHERE entityid = 'urn:ngsi-ld:Positions:positions'
ORDER BY ts DESC
LIMIT 100;
```

Si la consulta no devuelve datos, no significa que el mapper esté fallando: comprueba primero que el tópico ROS 2 publica, que Orion-LD ha cargado la configuración, que el dominio DDS coincide y que `-troe` está activo.

### 10.6. Qué sucede con `dds-context.jsonld` y los cambios posteriores

El **Compose de FIWARE monta `dds-config.json`**, pero **no publica ni carga automáticamente** `dds-context.jsonld`. Este segundo archivo define las IRIs y puede utilizarse como contexto JSON-LD de las peticiones NGSI-LD o servirse desde una URI HTTP, según cómo se configure la aplicación y la opción `-duc` de Orion-LD. Guardarlo en disco no equivale a hacerlo accesible por HTTP.

Si modificas los mappings y vuelves a generar `dds-config.json`, **reinicia el servicio `orion`** para que lea la nueva configuración de inicio:

```bash
docker compose -f docker-compose-fiware.yml up -d --force-recreate orion
```

Puedes tener ambos Compose activos a la vez. Como el nuevo Compose define `name: arise-fiware`, sus servicios y volúmenes tienen un proyecto distinto. Aun así, los procesos que usan `network_mode: host` comparten los puertos del sistema.

### 10.7. Límites de esta configuración de demostración

- Los nombres de usuario y contraseñas `orion/orion` son **credenciales de ejemplo**. No reutilices este Compose sin endurecerlo si se va a publicar fuera de un laboratorio o una red controlada. Cambiarlas requiere actualizar **PostgreSQL, Orion-LD y `datasources.yaml`** de forma coherente.
- La imagen `timescale/timescaledb-postgis:1.7.5-pg12` usa versiones antiguas de PostgreSQL/TimescaleDB porque reproduce la base del tutorial citado. Es necesario evaluar versiones actualizadas para producción y comprobar la compatibilidad con Orion-LD; también hay que verificar la **arquitectura de CPU** admitida por las imágenes si se utiliza un equipo ARM64.
- `privileged: true`, `ipc: host` y `network_mode: host` reproducen el enfoque de ARISE, pero conceden más permisos y acceso a recursos que una configuración endurecida. Revisa esos permisos antes de desplegar fuera de un entorno local de pruebas.
- El puerto `5433` de TimescaleDB se publica **solo en `127.0.0.1`**, para que Orion-LD pueda acceder sin exponer PostgreSQL directamente en todas las interfaces de red. Adminer y Grafana acceden a TimescaleDB por el nombre de servicio `timescale-db`.
- Los nombres de imagen con `latest` pueden cambiar; para obtener despliegues idénticos con el paso del tiempo conviene fijar etiquetas y versiones.

---

## 11. Comandos útiles

El script acepta:

```bash
bash start-mapping.sh           # Arrancar y construir si hace falta
bash start-mapping.sh status    # Ver estado de los contenedores
bash start-mapping.sh logs      # Seguir los logs de ambos servicios
bash start-mapping.sh stop      # Parar y eliminar los contenedores del Compose
bash start-mapping.sh rebuild   # Reconstruir sin caché y levantar de nuevo
```

Para obtener logs de un servicio específico:

```bash
docker logs --tail 50 ddsenabler-discovery
docker logs --tail 50 dds-ngsi-mapper-web
```

**Los JSON generados no se borran** al ejecutar `stop`, porque están en una carpeta del equipo anfitrión. Ese comando solo detiene el Compose del **mapper**; para detener FIWARE utiliza `docker compose -f docker-compose-fiware.yml down`.

## 12. Problemas frecuentes

| Mensaje o síntoma | Motivo habitual | Qué hacer |
| --- | --- | --- |
| `Failed to bind to port 8081` | El puerto WebSocket ya está ocupado. | Comprueba `ss -ltn | grep ':8081'` y elige otro puerto libre; actualiza también `DDS_DISCOVERY_URL`. |
| `EADDRINUSE :::3001` | La interfaz web intenta usar un puerto ocupado. | Cambia `WEB_PORT` a otro puerto libre. |
| `Configuration file path is empty` | Se lanzó discovery sin una ruta YAML. | Comprueba que el Compose pasa `/tmp/discovery-config.yml` como primer argumento. |
| `libddspipe_yaml.so.1: cannot open shared object file` | No se ha cargado el entorno compilado con `colcon`. | Comprueba que el comando ejecuta `. /ws/install/setup.sh`. |
| `WebSocket discovery timed out ... (no data)` | El mapper no recibe entradas de discovery a tiempo. | Comprueba los logs, el puerto, el dominio DDS y que haya un publicador ROS 2 activo. |
| `EACCES: permission denied, open 'out/dds-config.json'` | El usuario del contenedor no puede escribir en el volumen de salida. | Arranca con `bash start-mapping.sh` y comprueba `ls -ld fiware/out` y `ls -l fiware/out`. |
| En WSL2 la web responde dentro de Linux, pero no en el navegador de Windows | Problema de red o reenvío de `localhost`. | Comprueba `curl -I http://localhost:3001` desde WSL2, la integración de Docker Desktop, **Enable host networking** y la configuración de red de WSL2. |
| Docker da `permission denied` al conectarse al daemon | El usuario no tiene acceso a Docker. | Solicita al administrador que habilite el acceso; no intentes modificar permisos del daemon por tu cuenta. |
| Adminer no puede arrancar (`8082` ocupado) | Otro servicio utiliza el puerto del host. | Comprueba `ss -ltn | grep ':8082'`; cambia el mapeo de Adminer a otro puerto libre. |
| Error de publicación de `5433` | El puerto publicado de TimescaleDB está ocupado. | Ajusta `127.0.0.1:5433:5432` y **también** `ORIONLD_TROE_PORT` en Orion-LD. |
| Error al montar `/root/.orionld` | `fiware/out/dds-config.json` aún no existe o es un directorio. | Genera la configuración desde la web y comprueba `test -f fiware/out/dds-config.json`. |
| Grafana informa `Plugin not found` para Infinity | El plugin no se instaló o no es compatible con la imagen. | Revisa `docker compose -f docker-compose-fiware.yml logs grafana` y `GF_INSTALL_PLUGINS`. |
| Orion-LD responde pero `public.attributes` está vacío | Todavía no se han recibido o persistido muestras DDS. | Comprueba la publicación ROS 2, el dominio `0`, los logs de Orion y la conexión TRoE. |

Para diagnosticar primero el estado general de **ambas etapas**:

```bash
bash start-mapping.sh status
docker compose -f docker-compose-fiware.yml ps
bash start-mapping.sh logs
# En otra terminal, si es necesario:
docker compose -f docker-compose-fiware.yml logs -f
```

## 13. Consideraciones sobre Linux y WSL2

- **Linux nativo:** `network_mode: host` comparte la red y los puertos del propio equipo. Si el puerto configurado ya está ocupado, el servicio no podrá iniciarse.
- **WSL2 con Docker Desktop:** utiliza contenedores Linux, habilita la integración con tu distribución WSL2 y activa **Enable host networking** (Docker Desktop 4.34 o posterior). Esta configuración es importante porque ambos Compose usan red host. Consulta la [documentación oficial de Docker](https://docs.docker.com/engine/network/drivers/host/).
- **Acceso a la web desde Windows:** normalmente puedes abrir `http://localhost:3001`, `http://localhost:8082` y `http://localhost:4430` desde el navegador de Windows. Si no funcionan, comprueba primero la respuesta dentro de WSL2 y después el [reenvío de localhost y la red de WSL2](https://learn.microsoft.com/es-es/windows/wsl/networking).
- **Descubrimiento DDS/ROS 2:** si el publicador se ejecuta en otra máquina, en Windows o en un entorno distinto al de los contenedores, revisa el dominio DDS, la conectividad y el tráfico multicast. WSL2 con NAT puede requerir ajustes; el modo de red reflejado (*mirrored*) puede mejorar la compatibilidad, pero no garantiza por sí solo el discovery.
- **Seguridad:** la interfaz del mapper no añade autenticación ni HTTPS y el Compose de FIWARE contiene credenciales de demostración. No publiques esos puertos fuera del equipo sin revisar la configuración.
- **Versiones reproducibles:** el Dockerfile de discovery descarga una rama pública y el mapper se construye desde `main`. Conviene fijar commits o versiones para despliegues repetibles.
- **Orden de arranque:** `depends_on` no garantiza que DDS-Enabler haya terminado de inicializarse. Espera a que discovery esté operativo antes de pulsar **Load discovery**.

## 14. Referencias

- [ARISE Framework — Calling ROS 2 Actions from FIWARE Context Broker](https://arise-framework-documentation.readthedocs.io/en/latest/rst/tutorials/ros2_fiware_actions_app.html) — **fuente principal de la configuración de Orion-LD, MongoDB y TimescaleDB**.
- [FIWARE Orion-LD — DDS User Guide](https://github.com/FIWARE/context.Orion-LD/blob/develop/doc/dds/orion-ld-dds.md) — explicación de `-wip dds`, `~/.orionld` y el mapping DDS ↔ NGSI-LD.
- [eProsima — DDS-Enabler (GitHub)](https://github.com/eProsima/DDS-Enabler) — discovery DDS.
- [DDS-NGSI-LD Mapper (GitHub)](https://github.com/Engineering-Research-and-Development/dds-ngsild-mapper) — interfaz de generación de mappings.
- [Grafana — Provisioning](https://grafana.com/docs/grafana/latest/administration/provisioning/) — configuración declarativa de `datasources.yaml`.
- [Grafana Infinity — instalación](https://grafana.com/docs/plugins/yesoreyeram-infinity-datasource/latest/installation/) — instalación del plugin de fuentes de datos externas.
- [Adminer — imagen oficial en Docker Hub](https://hub.docker.com/_/adminer) — acceso web a PostgreSQL.
- [TimescaleDB PostGIS — imágenes en Docker Hub](https://hub.docker.com/r/timescale/timescaledb-postgis/tags) — imagen y etiqueta utilizadas por el tutorial.
- [Docker Compose — documentación oficial](https://docs.docker.com/compose/).
- [Docker — red host y soporte en Docker Desktop](https://docs.docker.com/engine/network/drivers/host/) — requisito del Compose en Docker Desktop.
- [Microsoft — redes y acceso a `localhost` con WSL2](https://learn.microsoft.com/es-es/windows/wsl/networking) — conexiones entre el entorno Linux y el navegador de Windows.

---

**Resultado del despliegue completo:** el Mapping Tool genera `dds-config.json` y `dds-context.jsonld`; después, el Compose de FIWARE inicia Orion-LD con el mapping DDS configurado, MongoDB para el estado actual, TimescaleDB para el histórico y Adminer/Grafana para consultar o visualizar la información. Los datos solo aparecerán si Orion-LD recibe correctamente las comunicaciones DDS/ROS 2.
