# Plan de organización — Autopiloto ASV PINV (ROS 2)

Nov 10, 2026 · @Jose

## Contexto

La tesis entrega una placa autopiloto con GNC sobre ROS 2 para el ASV del LSD (FIUNA/CITEC), dentro del proyecto PINV. Autores: Jose Ferreira y Adrian Genes; tutor: Dr. Ing. Mario Arzamendia. Meta de entrega: fin de 2026.

Los objetivos del anteproyecto que este plan debe cubrir:

- Simulación del GNC en Gazebo.
- ROS 2 instalado y configurado en la placa de navegación (Raspberry Pi 5).
- Nav2 haciendo las tareas de GNC.
- Fusión de sensores (IMU + GPS RTK) para precisión y robustez.
- Arquitectura modular, para que futuros trabajos del laboratorio se enchufen fácil.
- Pruebas de validación en el ASV.

| Decisión | Valor |
| --- | --- |
| Bote | Catamarán \~2 × 1 m, hasta \~5 m/s |
| Propulsión | 2 motores diferenciales, sin reversa, lazo abierto |
| Sensores | IMU ICM-20948 en el ESP32; GPS RTK (a definir por Adrian) |
| Detección de obstáculos | No; área libre grande |
| Traducción de /cmd\_vel | En el ESP32 |
| Objetivos | En metros, origen = posición inicial del bote; waypoints deseados |
| Lenguaje de nodos | Python |
| Simulación | Sí, parte del proceso (WAM-V en Gazebo) |
| Métrica de evaluación | Precisión al llegar al objetivo |
| Lugar de pruebas | Lago |
| Enlace con tierra | Probablemente WiFi a laptop; LoRa en evaluación |

## Diagnóstico

El firmware del ESP32 está más avanzado que el workspace ROS: ya publica la IMU fusionada y escucha `/cmd_vel`. El workspace todavía está armado para el WAM-V simulado y no tiene una separación clara entre simulación y bote real.

### Firmware (programas de Adrian)

| Programa | Qué hace | Estado |
| --- | --- | --- |
| `Calibracion_IMU` | Calibra gyro, acel y magnetómetro (bias, escala, matriz de hierro duro y blando), estima varianzas y guarda todo en flash (`Preferences`) | Completo |
| `Prueba_IMU_con_uROS` | Madgwick 9-DOF a 50 Hz (cae a 6-DOF si el campo magnético es inválido), publica `/imu/data` best-effort con covarianzas, `frame_id = imu_link`, se reconecta solo al agente, suscribe `/cmd_vel` con watchdog de 500 ms | Funciona; falta mover los motores (`updateMotors()` es un TODO) |
| `Prueba_de_motores` | Rampa un ESC a 50 Hz entre 1500 y 1700 µs en GPIO4 | Prueba aislada, un solo motor |

Puntos a revisar en el firmware:

- `WHEEL_SEPARATION = 0.40` es un valor de ejemplo; en un catamarán de 1 m de ancho es la distancia real entre ejes de los motores.
- La cinemática puede dar velocidades negativas en una rueda; sin reversa hay que recortarlas a 0.
- El yaw de Madgwick está referido al norte magnético; `robot_localization` espera ENU (0 = este, antihorario positivo). Hay que verificarlo con el bote apuntando a un rumbo conocido.
- Todo está en un solo `.ino` de 845 líneas; los pines (ESC en GPIO4, LED) no están centralizados.

### Workspace `pinv_ws`

- `sim_utilities` mezcla adaptadores del WAM-V con la localización y la navegación del bote real.
- `pinv_controller` está vacío y ya no hace falta: la traducción de `/cmd_vel` vive en el ESP32.
- `pinv.sdf` es el robot con ruedas del tutorial de Nav2; EKF y Nav2 usan `wamv/wamv/base_link`.
- El mapa se publica con un nodo propio y una TF fija `map → odom` en (20, 20); con el origen en la posición inicial del bote debería ser identidad.
- `build/`, `install/` y `log/` dentro de `src/`, sin dependencias en los `package.xml`, un solo commit (agosto 2025) y el firmware fuera del repo.

## Arquitectura propuesta

Seis paquetes con una responsabilidad cada uno, más el firmware en el mismo repo. La regla que los ordena: todo lo que está por encima del contrato (`/imu/data`, `/gps/fix`, `/cmd_vel`) es idéntico en simulación y en el bote; solo cambia la capa de abajo.

&#91;embedded content: arquitectura propuesta · paquetes, nodos y topics\]

La flecha resaltada es la única salida hacia los motores: si Nav2, la Pi o el enlace se caen, el watchdog del ESP32 frena en 500 ms.

| Paquete | Tipo | Contiene | Viene de |
| --- | --- | --- | --- |
| `pinv_description` | ament\_cmake | URDF/xacro del catamarán: `base_link`, `imu_link`, `gps_link`, dos motores | `pinv_description` (se rehace el modelo; se borra el SDF con ruedas) |
| `pinv_bringup` | ament\_python | `real.launch.py`, `sim.launch.py`, agente micro-ROS, `foxglove_bridge`, script de rosbag | Los dos launch actuales |
| `pinv_localization` | ament\_python | `ekf.yaml`, `navsat.yaml`, `localization.launch.py` | `sim_utilities/config` |
| `pinv_navigation` | ament\_python | `nav2_params.yaml`, `navigation.launch.py`, mapa vacío grande o costmap rolling | `sim_utilities` (`empty_map_pub` se reemplaza por `map_server` o se elimina) |
| `pinv_mission` | ament\_python | Nodos `waypoint_mission`, `return_home`, `goal_evaluator`; misiones en YAML en metros | Nuevo |
| `pinv_sim` | ament\_python | Nodos `wamv_sensor_bridge` y `wamv_thrust_bridge` (emula sin reversa y timeout) | `republisher_node`, `cmd_node` |

`pinv_controller` y `cmd_translator` se eliminan, porque la traducción de velocidades ya está en el ESP32. Los scripts de PNG a mapa pasan a `pinv_navigation/scripts/`.

Frames TF según REP-105: `map → odom` (identidad, origen = donde arranca el bote), `odom → base_link` (EKF) y `base_link → imu_link / gps_link` (URDF).

## Firmware ESP32 y contrato con ROS

El ESP32 sigue siendo la placa de bajo nivel: lee la IMU, recibe `/cmd_vel` y mueve los ESC. Lo único que ROS necesita saber de él es este contrato; mientras se cumpla, la Pi y el firmware se pueden cambiar por separado.

| Topic | Tipo | Dirección | Frame | Frecuencia | Notas |
| --- | --- | --- | --- | --- | --- |
| `/imu/data` | `sensor_msgs/Imu` | ESP32 → Pi | `imu_link` | 50 Hz | Best-effort, orientación en ENU, covarianzas cargadas |
| `/cmd_vel` | `geometry_msgs/Twist` | Pi → ESP32 | — | ≥ 10 Hz | `linear.x` \[m/s\], `angular.z` \[rad/s\]; sin mensaje por 500 ms = motores a 0 |
| `/gps/fix` | `sensor_msgs/NavSatFix` | Driver GPS → Pi | `gps_link` | 5–10 Hz | Lo publica el driver RTK (ver Pendientes) |
| `/motors/output` (opcional) | `std_msgs/Float32MultiArray` | ESP32 → Pi | — | 10 Hz | µs enviados a cada ESC; sirve para depurar y para la tesis |

Organización propuesta del sketch principal, en la carpeta `firmware/` del mismo repo:

```
firmware/
├── pinv_autopilot/          ← el firmware que va al bote
│   ├── pinv_autopilot.ino   ← setup() y loop() cortos
│   ├── config.h             ← pines, separación de motores, límites de ESC, timeouts
│   ├── imu.h / imu.cpp      ← lectura, calibración y Madgwick
│   ├── motors.h / motors.cpp← cinemática, recorte sin reversa, µs de ESC, watchdog
│   └── ros_comm.h / .cpp    ← entidades micro-ROS y reconexión
├── calibracion_imu/         ← Calibracion_IMU tal cual
└── pruebas/
    └── motores/             ← Prueba_de_motores
```

Para completar `updateMotors()`:

1. Calcular `vL` y `vR` con la cinemática diferencial que ya existe.
2. Recortar cada una a `[0, V_MAX]`, porque los motores no van en reversa.
3. Mapear `0 → 1500 µs` y `V_MAX → ESC_MAX` (hoy 1700 µs), en dos pines LEDC a 50 Hz.
4. Con el watchdog vencido o sin agente, escribir 1500 µs en ambos ESC.

Al ser lazo abierto, `V_MAX` es una calibración empírica: medir con el GPS la velocidad del bote a `ESC_MAX` en el lago y ponerla en `config.h`.

## Plan de migración

Siete fases en orden, cada una termina con algo que funciona y un commit. Con la entrega a fin de año quedan unas 7 semanas, así que las fases 1–5 son el núcleo y la 6 se puede recortar.

1. **Limpieza y punto de partida (día 1).** Commit del estado actual, borrar `src/build`, `src/install`, `src/log` y el `pinv.urdf` de la raíz, agregar `.gitignore` (`build/ install/ log/`) y la carpeta `firmware/` con los tres programas. Listo cuando: `colcon build` compila limpio en la Pi.
2. **Estructura nueva sin cambiar comportamiento (semana 1).** Crear los paquetes, mover archivos y configs a su lugar, borrar `pinv_controller` y el `navigation_launch.py` sin uso. Listo cuando: la simulación del WAM-V navega igual que antes con `ros2 launch pinv_bringup sim.launch.py`.
3. **Contrato común sim/real (semana 1–2).** Los adaptadores de `pinv_sim` publican `/imu/data` y `/gps/fix` y emulan el ESP32 (sin reversa, timeout 500 ms). Frames renombrados a `base_link`. Listo cuando: los configs de localización y Nav2 son los mismos para sim y real, solo cambia `use_sim_time`.
4. **Firmware con motores (semana 2, en paralelo con Adrian).** Refactor en archivos y `updateMotors()` con dos ESC. Listo cuando: con `teleop_twist_keyboard` en la Pi los dos motores responden en banco y se frenan al cortar el teleop.
5. **Localización y Nav2 reales (semanas 3–4).** Primero EKF solo con IMU (verificar rumbo ENU), después IMU + GPS. Probar caminando con la placa y luego con el bote en tierra. Listo cuando: un objetivo de 20 m produce ruta y `/cmd_vel` coherentes.
6. **Misión y evaluación (semana 4–5).** `waypoint_mission`, `return_home` y `goal_evaluator` + script de rosbag. Listo cuando: una misión de 3 puntos en simulación termina en casa y deja un CSV con los errores.
7. **Pruebas en el lago (semanas 5–7).** Calibrar `V_MAX`, ajustar RPP y tolerancias, correr las misiones de validación y juntar los datos para el libro.

## Pendientes y decisiones abiertas

Cinco decisiones cambian partes del plan; las demás se resuelven en las pruebas.

| Tema | Recomendación | Por qué | Afecta |
| --- | --- | --- | --- |
| Dónde se conecta el GPS RTK | USB directo a la Pi, con su driver ROS y un cliente NTRIP | Las correcciones RTK llegan por internet; en la Pi ya hay red y drivers mantenidos. En el ESP32 habría que reenviar RTCM a mano | `pinv_localization`, firmware |
| Enlace con la laptop | WiFi con `foxglove_bridge` en la Pi | DDS por WiFi es pesado y se cae con la distancia; un solo websocket es más estable y Foxglove sirve como panel de la misión | `pinv_bringup` |
| Qué pasa si se pierde el enlace | `return_home` vuelve al origen tras N s sin heartbeat de la laptop o al pasar el tiempo de misión | Encaja con la idea del nodo de regreso; el watchdog del ESP32 ya frena si la Pi deja de mandar `/cmd_vel` | `pinv_mission` |
| Giro sin reversa | En RPP: `use_rotate_to_heading: false`, `allow_reversing: false` y un radio mínimo de giro razonable | Sin reversa el bote no gira en el lugar; RPP sigue la ruta con curvas suaves, que es por lo que conviene mantenerlo | `nav2_params.yaml` |
| Convención del rumbo | Verificar que el yaw del ESP32 sea ENU y cargar la declinación magnética del lago | Un rumbo girado 90° o con la declinación mal desplaza todo el GPS en el mapa | Firmware, `navsat.yaml` |

También falta definir: el driver y los ESC exactos (Adrian confirma), la distancia real entre motores y la velocidad máxima útil en el lago.
