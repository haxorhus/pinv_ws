# Informe de trabajo — 12/11/2026

ASV PINV · Raspberry Pi 5 (Ubuntu 24.04, ROS 2 Jazzy) · ESP32-S3 con micro-ROS · GPS RTK simpleRTK2B (ZED-F9P)

## Resumen

| # | Tema | Estado |
|---|------|--------|
| 1 | Firmware en un solo archivo (`pinv_autopilot_simple`) | Hecho, sin compilar |
| 2 | micro-ROS no creaba nodos ni tópicos | **Resuelto** (opción del IDE) |
| 3 | `/imu/data` publica a 10 Hz en vez de 50 Hz | Arreglo hecho, **falta flashear y medir** |
| 4 | GPS por USB en la Pi + reglas udev | Regla instalada; camino del driver en la Pi **descartado** |
| 5 | GPS RTK integrado al firmware (`/gps/fix`) | Código hecho, **sin compilar ni probar** |

---

## 1. Firmware en un solo archivo

Se creó `firmware/pinv_autopilot_simple/pinv_autopilot_simple.ino`, una versión del
autopiloto con todo en un `.ino` y separada en secciones (configuración, motores, IMU,
micro-ROS, `setup`/`loop`). Debería comportarse igual que `pinv_autopilot/`, que sigue
siendo la versión principal y modular.

- Los `struct` y el `enum` van arriba de todo para que no fallen los prototipos que
  genera automáticamente el IDE de Arduino.
- **No tiene el GPS** (punto 5). Hay que agregárselo cuando la versión modular funcione.
- Commit `2c5a242`.

## 2. micro-ROS: la placa no creaba nodos

**Síntoma:** el agente quedaba en `running...` y nunca aparecía `session established`.

**Diagnóstico:**
- La parte de micro-ROS del firmware nuevo es casi igual a la de `Prueba_IMU_con_uROS`,
  que sí conectaba.
- Se escuchó `/dev/ttyACM0` desde la Pi con el agente parado. La placa arrancaba bien
  (`rst:0x15 USB_UART_CHIP_RESET`, el reinicio normal al abrir el puerto), pero en ~35 s
  **no mandó ni un byte** por el USB.

**Causa:** en el Arduino IDE estaba **USB CDC On Boot: Disabled**. Con eso, `Serial`
(el transporte de micro-ROS) sale por los pines de UART0 y no por el USB.

**Solución:** *Tools → USB CDC On Boot: Enabled* y *USB Mode: Hardware CDC and JTAG*.
Después de eso se crearon el participante, los dos tópicos, el publisher y el subscriber.

## 3. Verificación del firmware y frecuencia de la IMU

Con el agente andando se corrieron `ros2 node info` e `imu_stats.py`:

| Chequeo | Resultado |
|---|---|
| Nodo `/esp32s3_imu_node` | Pub `/imu/data`, Sub `/cmd_vel` ✅ |
| Timestamps | 0 no crecientes (antes 490 de 500) ✅, retraso medio 3 ms |
| Gravedad | `acc z = 9.75 m/s²` ✅, cuaternión normalizado, `frame_id imu_link` |
| **Frecuencia** | **10.4 Hz** ❌ (esperado 50 Hz), `dt max` 119 ms |
| Yaw | −45.7° (falta saber hacia dónde apuntaba la placa; paso 5 de `pendientes.md`) |

**Causa probable de los 10 Hz:** `motorsUpdate()` llamaba a `ledcWrite()` dos veces por
vuelta de `loop()`, aunque el valor no cambiara. Con el core 3.x y la señal a 50 Hz,
cada escritura puede quedarse esperando al fin del período (hasta 20 ms). La lectura
de la IMU es igual a la del programa viejo, que sí llegaba a 50 Hz.

**Arreglo:** ahora los ESC se escriben solo cuando cambia el pulso. Está en
`pinv_autopilot/motors.cpp` y en `pinv_autopilot_simple.ino`. Para los motores no cambia
nada: la señal es la misma.

**Pendiente:** flashear y volver a correr `python3 firmware/pruebas/ros/imu_stats.py 10`.
Si no da ~50 Hz, la causa es otra.

## 4. GPS por USB en la Pi y reglas udev

**Diagnóstico:**

| Dispositivo | USB ID | Puerto hoy |
|---|---|---|
| u-blox ZED-F9P (puerto "USB GPS") | `1546:01a9` | `/dev/ttyACM1` |
| ESP32-S3 (USB-Serial-JTAG) | `303a:1001`, serie `20:6E:F1:FB:73:2C` | `/dev/ttyACM0` |

- Llega NMEA del GPS, pero en ese momento **no había fix** (GGA con calidad 0). Veía
  satélites con 25–49 dB-Hz: probablemente estaba bajo techo o recién encendido.

**Regla udev** (`udev/99-tesis.rules` en el repo, instalada en `/etc/udev/rules.d/`):
- Crea `/dev/gps_rtk` y `/dev/esp32` con `MODE=0666`.
- Hace que **ModemManager** ignore los dos puertos (está activo en la Pi).
- `/dev/gps_rtk → ttyACM1` ✅ funciona.
- `/dev/esp32` **no se pudo verificar** porque el ESP32 estaba desconectado. Revisar
  `ls -l /dev/esp32` cuando se vuelva a enchufar.

**Driver `ublox_gps` en la Pi:** se descartó porque el GPS va a ir por el ESP32
(punto 5). No se instaló `ros-jazzy-ublox` ni se creó el paquete `pinv_gps`.
Si algún día se retoma: el índice de apt de ROS era del 6/10 y daba `Hash Sum mismatch`,
así que primero hay que correr `sudo apt update`.

**Agente micro-ROS:** sigue con `/dev/ttyACM0`. El cambio propuesto, que todavía no se
aplicó, es:
```bash
docker run -it --rm -v /dev:/dev --privileged --net=host \
  microros/micro-ros-agent:jazzy serial --dev /dev/esp32
```

## 5. GPS RTK en el firmware (`/gps/fix`)

El simpleRTK2B le pasa sus datos al ESP32 por UART1. Las correcciones RTCM siguen
llegando por radio a la UART2 del GPS, que no se tocó.

**Decisiones:**
- Se mantuvo el patrón del firmware actual: lazo único en `loop()`, **sin tareas
  FreeRTOS, sin mutex y sin timers nuevos**.
- **Pines:** GPIO18 está ocupado (es el SCL de la IMU). Quedaron RX=16 y TX=17 como
  **provisorios (`PENDIENTE` en `config.h`)**.
- **Altitud** sobre el elipsoide WGS84 (`getAltitude()`), que es lo que define `NavSatFix`.
- **Reloj:** el mismo `nowNs()` sincronizado que usa la IMU (mismo reloj para el EKF).

**Archivos:**
- `gps.h` / `gps.cpp` (nuevos):
  - UART1 a 38400 con `SFE_UBLOX_GNSS_SERIAL`.
  - Al conectar configura `setUART1Output(UBX)`, `setNavigationFrequency(5)` y
    `setAutoPVT(true)`, solo en RAM/BBR (sin `saveConfiguration`).
  - `begin()` se reintenta cada 1 s, solo si llegan bytes por la UART, para no frenar el
    lazo cuando el GPS está desconectado.
  - Si pasan 3 s sin PVT, se reconecta.
- `ros_comm.*`:
  - Publisher best-effort de `sensor_msgs/NavSatFix` en `/gps/fix` (`gps_link`,
    `service = 15`, covarianza diagonal `[hAcc², hAcc², vAcc²]`, `DIAGONAL_KNOWN`).
  - Estado: sin fix → `NO_FIX`; `carrSoln > 0` → `GBAS_FIX` (2); si no → `FIX`.
- `pinv_autopilot.ino`: llama a `gpsInit()` y en `loop()` publica cuando llega un PVT nuevo.
- `config.h`: pines, baud, frecuencia y tiempos del GPS.

**Verificado:** el uso de la API de `NavSatFix` se compiló y se corrió contra los headers
de Jazzy instalados en la Pi.

**Sin verificar:** la compilación completa para el ESP32. En la Pi no hay `arduino-cli`
ni las librerías.

**Respaldo** del sketch antes del GPS: `firmware/backups/pinv_autopilot_2026-11-12_antes_gps/`.

---

## Próximos pasos

1. Confirmar los pines del GPS (RX/TX) y corregirlos en `config.h`.
2. Instalar **SparkFun u-blox GNSS v3** desde el Library Manager.
3. Compilar `pinv_autopilot`. Si hay errores, copiarlos completos.
4. Flashear (con las hélices sacadas) y verificar:
   - `python3 firmware/pruebas/ros/imu_stats.py 10` → ~50 Hz
   - `ros2 topic hz /gps/fix` → ~5 Hz
   - `ros2 topic echo /gps/fix --once` → `status.status = 2` con RTK
5. Si `gnss.begin()` no conecta: revisar TX/RX cruzados, baud (probar 115200), IOREF en
   3.3 V con GND común, y que UART1 tenga salida activada (u-center).
6. Enchufar el ESP32, verificar `/dev/esp32` y pasar el agente a ese symlink.
7. Agregar el GPS a `pinv_autopilot_simple.ino`.
8. Rumbo: seguir con el paso 5 de `pendientes.md`.
