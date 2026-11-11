# Pendientes — prueba del firmware `pinv_autopilot` (Fase 4)

Rama `reorganizacion`. El firmware nuevo está en `firmware/pinv_autopilot/`
(`config.h`, `imu.*`, `motors.*`, `ros_comm.*`). **No se compiló todavía**: en la
Pi no hay toolchain de Arduino. Lo que sí se probó en la Pi: la lógica de
motores (compilada para PC con un `Arduino.h` simulado, 13 casos) y la
cuenta de la rotación a ENU.

## Qué cambió respecto del firmware anterior

1. **Rumbo en ENU.** Madgwick da yaw 0 con la proa al norte magnético; ahora se
   rota +90° y se publica yaw 0 = este, 90 = norte (lo que espera
   `robot_localization`). La declinación se corrige en ROS (`navsat.yaml`).
2. **Ejes del magnetómetro.** El AK09916 tiene Y y Z invertidos respecto del
   acelerómetro; se usa `(x, -y, -z)` (`MAG_AXIS_FIX` en `config.h`). Se aplica
   después de la calibración, así que **no hace falta recalibrar**.
3. **Timestamp.** Antes avanzaba de a 1 s. Ahora se toma la hora del agente al
   sincronizar y se interpola con `esp_timer_get_time()` (µs). No publica la IMU
   hasta tener el reloj sincronizado.
4. **Motores.** Dos ESC por LEDC (50 Hz, 14 bits, XTAL, igual que
   `Prueba_de_motores`): `v`, `ω` → `vL`, `vR` → recorte a `[0, V_MAX]` (sin
   reversa) → 1500…1700 µs. Neutro durante los primeros 5 s (armado), sin agente,
   o sin `/cmd_vel` por 500 ms.

## Antes de flashear

- [ ] **Pin del ESC derecho.** En `config.h` quedó `PIN_ESC_RIGHT 5` provisorio
      (libre, no es pin de arranque ni USB del S3). El izquierdo es GPIO4, el de
      `Prueba_de_motores`. Confirmar con el cableado.
- [ ] **Separación entre motores.** `MOTOR_SEPARATION_M 0.75` provisorio. Medir
      entre los ejes de los dos motores y poner el mismo valor en
      `src/pinv_description/urdf/pinv.urdf.xacro` (`motor_separation`).
- [ ] `V_MAX_MPS 2.0` queda así hasta medirlo en el lago (Fase 7). Solo cambia la
      escala: con 2.0, `v = 1 m/s` → 1600 µs.

## 1. Compilar y flashear

- [ ] Abrir `firmware/pinv_autopilot/pinv_autopilot.ino` en el Arduino IDE (las
      pestañas `.h`/`.cpp` se abren solas).
- [ ] Mismo entorno con el que compilaban antes: core ESP32 **3.x** (lo pide
      `ledcAttach`), placa ESP32S3, *USB CDC On Boot* como lo tenían, y las
      librerías `micro_ros_arduino`, SparkFun ICM-20948 y la **misma** MadgwickAHRS
      de antes (la que tiene `getQuaternion`).
- [ ] Si no compila: copiar el error completo y pasármelo.
- [ ] Flashear. **Hélices sacadas o bote asegurado** desde acá en adelante.

LED: apagado = esperando agente · fijo = conectado y reloj sincronizado ·
parpadeo lento (0,5 s) = falta la calibración (correr `calibracion_imu`) ·
parpadeo rápido (0,1 s) = falló la configuración de los ESC.

## 2. Agente y tópicos

- [ ] Levantar el agente (el mismo `docker run` de hoy), por ejemplo:
      `docker run -it --rm --net=host -v /dev:/dev --privileged microros/micro-ros-agent:jazzy serial --dev /dev/ttyACM0 -b 115200`
- [ ] En otra terminal (`source /opt/ros/jazzy/setup.bash`):
      `ros2 node info /esp32s3_imu_node`
      → **Subscribers: `/cmd_vel`** y **Publishers: `/imu/data`**. Hoy faltaba el suscriptor.

## 3. Timestamps y frecuencia

- [ ] `python3 firmware/pruebas/ros/imu_stats.py 10`
      Esperado: `frecuencia ... 50 Hz`, **`stamps no crecientes: 0`** (hoy: 490 de
      500) y retraso stamp→recepción de pocos ms (hoy: 19–999 ms).

## 4. Gravedad (placa quieta y nivelada)

- [ ] `python3 .claude/skills/ros2-troubleshooting/scripts/check_imu_gravity.py --topic /imu/data --assume-aligned`
      Esperado: `[PASS]`, gravedad ~+9,8 en +Z (hoy ya pasaba).

## 5. Rumbo (brújula)

Lejos de metales, motores y la notebook. Ejecutar
`python3 firmware/pruebas/ros/imu_yaw.py` y apuntar la **flecha X de la placa**:

| Proa (eje X) hacia | yaw ENU esperado |
| --- | --- |
| Norte magnético | ~90° |
| Este | ~0° |
| Sur | ~−90° |
| Oeste | ~180° / −180° |

- [ ] Girar la placa en sentido **antihorario** (vista desde arriba): el yaw tiene
      que **aumentar**.
- [ ] Inclinar la placa ~20° sin girarla: el yaw no debería moverse más de unos grados.
- [ ] Anotar los 4 valores. Usar la brújula del celular en modo **norte
      magnético**; si está en norte verdadero, la diferencia es la declinación.

Si falla:
- yaw **gira al revés** o salta mucho al inclinar → poner `MAG_AXIS_FIX 0` en
  `config.h`, reflashear y repetir. Pasarme qué pasó en cada caso.
- yaw correcto pero **corrido un ángulo fijo** → anotarlo (puede ser montaje o
  declinación).
- `sigma_yaw` crece sin parar → el magnetómetro está fuera de 10–100 µT
  (interferencia): alejar la placa de metales/cables.

## 6. Motores en banco

**Seguridad:** sin hélices o bote amarrado fuera del agua, con alguien listo para
cortar la batería. Los ESC tienen que pitar el armado (5 s en neutro) al encender.

Publicar a 10 Hz (más rápido que el watchdog de 500 ms):

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

Pulsos esperados con `MOTOR_SEPARATION_M 0.75` y `V_MAX_MPS 2.0`:

| Comando | Izquierdo | Derecho | Qué se ve |
| --- | --- | --- | --- |
| `{linear: {x: 0.5}}` | 1550 µs | 1550 µs | ambos iguales, despacio |
| `{linear: {x: 1.0}}` | 1600 µs | 1600 µs | ambos más rápido |
| `{linear: {x: 0.5}, angular: {z: 1.0}}` | 1513 µs | 1588 µs | derecho más rápido (gira a babor) |
| `{linear: {x: 0.5}, angular: {z: -1.0}}` | 1588 µs | 1513 µs | izquierdo más rápido (gira a estribor) |
| `{angular: {z: 1.0}}` | 1500 µs | 1538 µs | solo el derecho, lento |
| `{linear: {x: -0.5}}` | 1500 µs | 1500 µs | **nada** (sin reversa) |

- [ ] Cada fila se comporta como dice la tabla.
- [ ] **Izquierdo y derecho físicamente correctos.** Si están cruzados, intercambiar
      `PIN_ESC_LEFT`/`PIN_ESC_RIGHT` en `config.h`.
- [ ] **Watchdog:** con un motor girando, cortar el `ros2 topic pub` (Ctrl+C) →
      se detiene en ≤ 0,5 s.
- [ ] **Sin agente:** con un motor girando, parar el contenedor del agente → se
      detiene (el LED se apaga).
- [ ] Opcional, con teclado: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`.
      Hay que **mantener** la tecla; al soltarla el watchdog frena (esperable).

## 7. Opcional: Nav2 moviendo los motores (sin GPS)

Con el agente de Docker andando (por eso `microros:=false`):

```bash
cd ~/pinv_ws && source install/setup.bash
ros2 launch pinv_bringup real.launch.py microros:=false
```

Sin GPS el EKF solo tiene rumbo, pero Nav2 arranca. Mandar un objetivo:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 20.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

- [ ] Los motores responden y el diferencial gira hacia el este (x = este en ENU).
      A los ~15 s Nav2 reporta "Failed to make progress" porque el bote no se
      mueve: es esperable en banco.

## Para pasarme cuando termines

- Salida de `ros2 node info /esp32s3_imu_node` y de `imu_stats.py 10`.
- Los 4 rumbos de la prueba de la brújula y si el sentido de giro fue correcto.
- Qué motor es cuál y si alguna fila de la tabla no coincidió.
- Pin real del ESC derecho y separación medida entre motores.

## Otros pendientes (no son para mañana)

- Medir la posición de la IMU y de la antena GPS respecto del centro del bote
  (`pinv.urdf.xacro`, hoy en 0, 0, 0).
- Declinación magnética del lago → `magnetic_declination_radians` en
  `src/pinv_localization/config/navsat.yaml`.
- Definir receptor GPS RTK, driver ROS y cliente NTRIP (Adrian).
- `real.launch.py` lanza el agente como paquete ROS (no instalado); con Docker
  usar `microros:=false`. Se puede hacer que el launch levante el contenedor.
- `.claude/` (skills de ROS 2) sigue sin commitear: decidir si va al repo.
