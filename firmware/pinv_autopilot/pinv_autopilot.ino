// =====================================================
// pinv_autopilot — Firmware de bajo nivel del ASV PINV (ESP32-S3)
//
//  - Lee el ICM-20948, aplica la calibración guardada por
//    calibracion_imu y estima la orientación con Madgwick (50 Hz).
//  - Publica /imu/data (sensor_msgs/Imu, ENU, best-effort) por micro-ROS
//    sobre USB serial, con timestamp sincronizado con el agente.
//  - Lee el GPS RTK (ZED-F9P) por UART1 y publica /gps/fix (NavSatFix,
//    5 Hz) con el mismo reloj que la IMU.
//  - Escucha /cmd_vel y mueve dos ESC diferenciales sin reversa.
//    Sin agente o sin /cmd_vel por 500 ms, los motores van a neutro.
//
// Parámetros: config.h. Módulos: imu, gps, motors, ros_comm.
// Librerías: micro_ros_arduino, SparkFun ICM-20948, MadgwickAHRS,
//            SparkFun u-blox GNSS v3.
//
// LED: apagado = esperando agente, encendido = conectado y sincronizado,
//      parpadeo lento = falta la calibración, rápido = falla de LEDC.
// =====================================================

#include "config.h"
#include "imu.h"
#include "gps.h"
#include "motors.h"
#include "ros_comm.h"

static void blinkForever(uint32_t periodMs)
{
    // Los ESC ya quedaron en neutro: LEDC sigue generando la señal.
    pinMode(PIN_LED, OUTPUT);
    while (true)
    {
        digitalWrite(PIN_LED, !digitalRead(PIN_LED));
        delay(periodMs);
    }
}

void setup()
{
    // Primero los ESC: neutro desde el encendido para que armen.
    const bool motorsOk = motorsInit();

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    if (!motorsOk)
        blinkForever(100);

    if (!imuLoadCalibration())
        blinkForever(500);

    imuInit();
    gpsInit();
    rosInit();
}

void loop()
{
    rosSpin();

    // IMU a IMU_UPDATE_HZ siempre (el filtro converge aunque no haya
    // agente); se publica solo con agente y reloj sincronizado.
    static uint32_t previousUs = 0;
    const uint32_t nowUs = micros();

    if ((uint32_t)(nowUs - previousUs) >= IMU_UPDATE_US)
    {
        previousUs = nowUs;

        ImuSample sample;
        if (imuUpdate(sample))
            rosPublishImu(sample);
    }

    // GPS: se lee en cada vuelta (no bloquea) y se publica cuando llega
    // un PVT nuevo (GPS_NAV_HZ).
    GpsSample fix;
    if (gpsUpdate(fix))
        rosPublishGps(fix);

    motorsUpdate(rosReady());

    digitalWrite(PIN_LED, rosReady());

    delay(1);
}
