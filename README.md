# Taller 3 — ROS 2 + InfluxDB + Grafana + Wireshark (README)

**Objetivo.** Montar un flujo completo de adquisición y monitoreo en tiempo real:
**Arduino → ROS 2 → InfluxDB → Grafana**, y capturar tráfico **RTPS/DDS** con **Wireshark** para comparar **QoS RELIABLE vs BEST_EFFORT**.

> **Nota del repositorio**
> Aquí se incluyen **únicamente los archivos de código clave** (nodos `.py`, `t3_qos.launch.py` y `docker-compose.yml`).
> **No** se sube el árbol completo autogenerado de `src/` (build, install, logs, etc.).
> Abajo te explico **dónde colocar cada archivo** y cómo **reconstruir el paquete** localmente.

---

## 1) Arquitectura

* **ros2_sniff**: contenedor con ROS 2 (nodos Python).
* **InfluxDB 2.x**: almacenamiento de series temporales (bucket `temperatures`, org `wsn`).
* **Grafana**: visualización (consulta Flux).
* **netshoot/tcpdump**: captura RTPS (UDP 7400–7500) para Wireshark.

---

## 2) Estructura del repo/local

```
taller3/
├─ docker-compose.yml                # InfluxDB + Grafana
├─ src/
│   └─ sensor_serial/
│        ├─ sensor_serial/
│        │  ├─ sensor_node.py
│        │  ├─ processor_node.py
│        │  ├─ monitor_node.py
│        │  └─ exporter_node.py
│        ├─ launch/
│        │  └─ t3_qos.launch.py
│        └─ setup.py
└─ pcap/                              # Se generará con las capturas
```

> **Qué viene en el repo**: los `.py`, el `launch` y `docker-compose.yml`.
> **Qué debes crear tú**: carpetas `sensor_serial/`, `launch/` y `setup.py` (plantilla abajo).

---

## 3) Requisitos

* Docker Desktop o Docker CE + Compose.
* Wireshark instalado en el host.
* (Opcional) Arduino enviando enteros a `/dev/ttyACM0` o `/dev/ttyUSB0`.

---

## 4) Despliegue de InfluxDB + Grafana

Desde la carpeta del proyecto:

```bash
docker compose up -d
```

* InfluxDB: [http://localhost:8086](http://localhost:8086) (usuario: `admin`, pass: `admin123`, org: `wsn`, bucket: `temperatures` si usas el compose provisto).
* Grafana: [http://localhost:3000](http://localhost:3000) (admin/admin la primera vez).

En InfluxDB, crea un **API Token** (`Load Data → Tokens`).

---

## 5) Preparar el paquete ROS 2 localmente

Dentro del contenedor ROS 2 (o en tu máquina si usas ROS 2 nativo), crea el esqueleto si no existe:

```bash
# Si aún no existe el paquete:
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python sensor_serial

# Crea carpetas
mkdir -p sensor_serial/sensor_serial sensor_serial/launch
```

**Copia los archivos del repo** a las rutas indicadas en la estructura del punto 2.


### 5.1 `setup.py` (instalar nodos y launch)

```python
from setuptools import setup

package_name = 'sensor_serial'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/t3_qos.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tu_nombre',
    maintainer_email='you@example.com',
    description='Taller 3',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sensor_node=sensor_serial.sensor_node:main',
            'processor_node=sensor_serial.processor_node:main',
            'monitor_node=sensor_serial.monitor_node:main',
            'exporter_node=sensor_serial.exporter_node:main',
        ],
    },
)
```

### 5.2 Dependencias Python en contenedor (PEP 668)

Dentro del contenedor ROS:

```bash
apt update && apt install -y python3-venv python3-serial
python3 -m venv /opt/ros-venv
source /opt/ros-venv/bin/activate
pip install --upgrade pip wheel
pip install influxdb-client
```

### 5.3 Compilar e instalar

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sensor_serial --symlink-install
source install/setup.bash
```

---

## 6) Conectar el contenedor ROS a la red del compose

Inspecciona la red creada por compose (ej. `taller3_proyecto`) y conecta tu contenedor ROS:

```bash
docker network connect taller3_proyecto ros2_sniff
docker exec -it ros2_sniff bash -lc "getent hosts influxdb; curl -sSf http://influxdb:8086/health"
```

Debe responder `{"status":"pass"...}`.

---

## 7) Variables de entorno para el exportador

En **ros2_sniff**:

```bash
export INFLUX_URL=http://influxdb:8086
export INFLUX_TOKEN=<TU_TOKEN_DE_INFLUX>
export INFLUX_ORG=wsn
export INFLUX_BUCKET=temperatures
```


---

## 8) Ejecución del sistema (una QoS a la vez)

### 8.1 Modo **RELIABLE**

En **ros2_sniff**:

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch sensor_serial t3_qos.launch.py reliability:=reliable
```

### 8.2 Modo **BEST_EFFORT**

Detén la anterior. Luego:

```bash
ros2 launch sensor_serial t3_qos.launch.py reliability:=best_effort
```

---

## 9) Grafana (panel Flux)

* Configura el **Data Source** InfluxDB en Grafana con **Flux**:

  * URL: `http://influxdb:8086`
  * Organization: `wsn`
  * Token: *(el que creaste)*
  * Default bucket: `temperatures`

* Crea un panel **Time series** con la consulta:

```flux
from(bucket: "temperatures")
  |> range(start: -30m)
  |> filter(fn: (r) => r._measurement == "temperature" and r._field == "celsius")
  |> aggregateWindow(every: 1s, fn: mean, createEmpty: false)
  |> yield(name: "mean")
```

---

## 10) Captura RTPS para Wireshark

Crea la carpeta `pcap/` y captura con **netshoot**:

```bash
# RELIABLE
docker run --rm --network taller3_proyecto \
  -v "$PWD/pcap:/pcap" \
  --cap-add=NET_ADMIN --cap-add=NET_RAW \
  nicolaka/netshoot \
  tcpdump -i eth0 -w /pcap/reliable_capture.pcap udp portrange 7400-7500
```

Corre la QoS RELIABLE por 60–90 s, luego **Ctrl+C** en el tcpdump.
Repite cambiando el archivo a `best_effort_capture.pcap` para el otro modo.

**En Wireshark**, abre el `.pcap` y filtra:

```
udp.port >= 7400 && udp.port <= 7500
```

Para control de fiabilidad:

```
rtps.submessageid == 0x06 || rtps.submessageid == 0x07   # ACKNACK / HEARTBEAT
```

> Si no aparecen, prolonga la captura o reinicia el lector durante la transmisión: en enlaces limpios, el control puede ser esporádico.

## 11) ¿Cómo se construyó la imagen del contenedor ROS?

Tienes dos opciones:

* **Commit del contenedor** (rápido):

  ```bash
  docker commit ros2_sniff heliosxs/ros2_sniff:jazzy-t3
  docker push heliosxs/ros2_sniff:jazzy-t3
  ```

* **Dockerfile reproducible** (recomendado): generar imagen desde código (no incluido aquí por brevedad).

> **Nota final — Imagen completa**
> Puedes descargar una imagen lista desde Docker Hub:
> **[https://hub.docker.com/repository/docker/heliosxs/ros2_sniff/general](https://hub.docker.com/repository/docker/heliosxs/ros2_sniff/general)**


¡Listo! Con esto puedes clonar el repo, colocar cada archivo en su ruta, levantar los servicios, ejecutar ambos modos de QoS, graficar en Grafana y capturar/analizar RTPS en Wireshark de forma reproducible.

> **Nota**
> La guia del taller muestra a detalle como conectar arduino con el contenedor docker.
