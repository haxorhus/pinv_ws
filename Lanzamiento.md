# Lanzamiento del bote — ayuda de memoria

Orden: **hardware → agente → chequeos → Nav2 → objetivo**. Cada bloque va en
su propia terminal de la Pi.

> **Seguridad:** hélices sacadas o bote amarrado mientras se prueba en tierra.
> Tener a alguien listo para cortar la batería.

---

## 0. Antes de empezar

- [ ] Batería conectada; al encender, los ESC pitan el armado (5 s en neutro).
- [ ] ESP32 conectado por USB a la Pi:
      ```bash
      ls /dev/ttyACM*        # tiene que aparecer /dev/ttyACM0 (o ACM1)
      ```
- [ ] Workspace compilado (solo si se cambió algo en `src/`):
      ```bash
      cd ~/pinv_ws && source /opt/ros/jazzy/setup.bash && colcon build
      ```

## 1. Agente micro-ROS (terminal 1)

```bash
docker run -it --rm --net=host -v /dev:/dev --privileged \
  microros/micro-ros-agent:jazzy serial --dev /dev/ttyACM0 -b 115200
```

Si el puerto es otro, cambiar `/dev/ttyACM0`. Dejar esta terminal abierta.

LED del ESP32:

| LED | Significado |
| --- | --- |
| apagado | esperando al agente |
| fijo | conectado y reloj sincronizado ✅ |
| parpadeo lento (0,5 s) | falta la calibración de la IMU (`calibracion_imu`) |
| parpadeo rápido (0,1 s) | falló la configuración de los ESC |

## 2. Chequear que el ESP32 publica bien (terminal 2)

```bash
source /opt/ros/jazzy/setup.bash
cd ~/pinv_ws

ros2 node info /esp32s3_imu_node
#   Subscribers: /cmd_vel    Publishers: /imu/data

python3 firmware/pruebas/ros/imu_stats.py 10
#   ~50 Hz y "stamps no crecientes: 0"

python3 firmware/pruebas/ros/imu_yaw.py      # opcional: rumbo ENU
#   proa al este ≈ 0°, al norte ≈ 90° (Ctrl+C para salir)
```

**Si no aparece `/imu/data`, no seguir:** sin IMU el EKF no publica la TF
`odom → base_link` y Nav2 se queda esperando.

## 3. Lanzar todo el GNC (terminal 3)

```bash
cd ~/pinv_ws && source install/setup.bash
ros2 launch pinv_bringup real.launch.py microros:=false
```

`microros:=false` porque el agente ya corre en Docker (paso 1).

Esto levanta:
- `robot_state_publisher` (modelo y TF `base_link → imu_link / gps_link`)
- `ekf_filter_node` + `navsat_transform` + TF fija `map → odom`
- `map_server` (mapa vacío 400 × 400 m, origen = donde arranca el bote)
- Nav2: `controller_server`, `planner_server`, `behavior_server`,
  `bt_navigator`, `waypoint_follower`

Esperar ~20 s hasta ver `Managed nodes are active` en el log.

## 4. Verificar que Nav2 quedó listo (terminal 4)

```bash
cd ~/pinv_ws && source install/setup.bash

ros2 lifecycle get /bt_navigator           # → active [3]
ros2 lifecycle get /controller_server      # → active [3]
ros2 run tf2_ros tf2_echo map base_link    # tiene que imprimir la transformada (Ctrl+C)
ros2 topic hz /odometry/filtered           # ~30 Hz (Ctrl+C)
```

## 5. Mandar objetivos

Coordenadas en **metros desde la posición inicial**, en ENU:
**x = este, y = norte**. La orientación final se ignora (sin reversa).

**Un punto:**
```bash
ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 20.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

**Varios puntos seguidos:**
```bash
ros2 action send_goal --feedback /navigate_through_poses nav2_msgs/action/NavigateThroughPoses \
  "{poses: [
    {header: {frame_id: map}, pose: {position: {x: 20.0, y: 0.0},  orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 20.0, y: 20.0}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 0.0,  y: 0.0},  orientation: {w: 1.0}}}
  ]}"
```

**Ver qué manda a los motores:**
```bash
ros2 topic echo /cmd_vel
```

En tierra (el bote no se mueve), a los ~15 s aparece `Failed to make progress`.
Es normal: Nav2 limpia los costmaps, espera 5 s y reintenta.

## 6. Parar

| Qué | Cómo | Resultado |
| --- | --- | --- |
| Cancelar el objetivo | `Ctrl+C` en la terminal del `send_goal` | Nav2 deja de mandar `/cmd_vel` → motores a neutro en ≤ 0,5 s |
| Apagar la navegación | `Ctrl+C` en la terminal 3 | igual: el watchdog del ESP32 frena en ≤ 0,5 s |
| Cortar la comunicación | `Ctrl+C` en la terminal 1 (agente) | el ESP32 pierde el agente → neutro, LED apagado |
| **Emergencia** | **cortar la batería** | — |

## 7. Manejo manual (opcional, sin Nav2)

Con el agente andando (paso 1). Se puede tener el launch del paso 3 corriendo
para ver la IMU/EKF, pero **sin mandar objetivos** (Nav2 y el teclado
pelearían por `/cmd_vel`).

```bash
cd ~/pinv_ws && source install/setup.bash
ros2 run pinv_teleop teleop_teclado
```

Modo acelerador: la velocidad queda fija hasta que la cambiás y se publica a
10 Hz (no hace falta mantener la tecla).

| Tecla | Acción |
| --- | --- |
| `w` / ↑ | acelerar (+0,1 m/s) |
| `s` / ↓ | desacelerar (−0,1 m/s, mínimo 0: sin reversa) |
| `a` / ← | girar a babor (+0,1 rad/s) |
| `d` / → | girar a estribor (−0,1 rad/s) |
| `c` | timón al centro |
| **ESPACIO** / `x` | **frenar todo** |
| `q` / Ctrl+C | salir (publica `cmd_vel = 0` antes) |

En pantalla muestra los µs estimados de cada ESC (misma cuenta que el firmware).
Pasos y límites por parámetro, por ejemplo:

```bash
ros2 run pinv_teleop teleop_teclado --ros-args -p paso_lineal:=0.2 -p v_max:=1.0
```

O a mano, a 10 Hz:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

## Si algo no anda

| Síntoma | Qué mirar |
| --- | --- |
| No aparece `/dev/ttyACM0` | cable USB / probar `/dev/ttyACM1` |
| LED apagado con el agente corriendo | puerto o baudrate del `docker run`; reiniciar el ESP32 |
| Nav2 no llega a `active` | ¿llega `/imu/data`? `ros2 topic hz /imu/data` |
| `tf2_echo map base_link` no imprime nada | el EKF no recibe la IMU (QoS o timestamps: correr `imu_stats.py`) |
| El bote gira para el lado equivocado | rumbo de la IMU mal (`imu_yaw.py`, `MAG_AXIS_FIX` en `config.h`) |
| Motores izquierdo/derecho cruzados | intercambiar `PIN_ESC_LEFT` / `PIN_ESC_RIGHT` en `config.h` |
| `Control loop missed its desired rate` seguido | Pi cargada: cerrar lo que no haga falta |

## Todavía falta (ver `pendientes.md`)

- Driver del GPS RTK en `real.launch.py`: hoy el EKF solo tiene rumbo, así
  que **no sabe dónde está el bote**. En el agua, sin GPS, no navega de verdad.
- Declinación magnética en `src/pinv_localization/config/navsat.yaml`.
- Medir separación de motores y posición de IMU/GPS en `pinv.urdf.xacro`.
