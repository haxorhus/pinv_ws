// =====================================================
// config.h — Todos los parámetros del firmware del ASV PINV
//
// Lo marcado "PENDIENTE" hay que confirmarlo en el bote.
// =====================================================
#pragma once

#include <Arduino.h>

// -----------------------------------------------------
// Pines (ESP32-S3)
// -----------------------------------------------------
#define PIN_LED         7       // LED de estado
#define PIN_IMU_SDA     8       // I2C ICM-20948
#define PIN_IMU_SCL     18
#define PIN_ESC_LEFT    4       // de Prueba_de_motores (ESC probado)
#define PIN_ESC_RIGHT   5       // PENDIENTE: confirmar con el cableado de Adrian

// -----------------------------------------------------
// IMU ICM-20948
// -----------------------------------------------------
#define IMU_AD0_VAL     0       // dirección I2C 0x68
#define IMU_I2C_HZ      400000
#define IMU_UPDATE_HZ   50
#define IMU_UPDATE_US   (1000000UL / IMU_UPDATE_HZ)

// El magnetómetro AK09916 dentro del ICM-20948 tiene X igual al
// acelerómetro/giróscopo pero Y y Z invertidos (hoja de datos del
// ICM-20948, orientación de ejes). 1 = corregir a (x, -y, -z).
// La calibración guardada está en ejes crudos del chip; la corrección
// se aplica DESPUÉS de calibrar, así que no hace falta recalibrar.
#define MAG_AXIS_FIX    1

// Rango de campo magnético válido [uT]. Fuera de esto Madgwick pasa a 6-DOF.
#define MAG_MIN_UT      10.0f
#define MAG_MAX_UT      100.0f

// Desviación estándar de la orientación publicada
#define IMU_RP_STD_DEG       3.0f
#define IMU_YAW_STD_DEG     10.0f

// -----------------------------------------------------
// Motores / ESC (datos de Prueba_de_motores)
// -----------------------------------------------------
#define ESC_PWM_FREQ_HZ     50
#define ESC_PWM_RES_BITS    14
#define ESC_PERIOD_US       (1000000UL / ESC_PWM_FREQ_HZ)   // 20000 us
#define ESC_NEUTRAL_US      1500    // motor detenido
#define ESC_MAX_US          1700    // máximo probado en banco
#define ESC_ARM_MS          5000    // neutro sostenido al encender para armar el ESC

// Distancia entre los ejes de los dos motores [m].
// PENDIENTE: medir en el catamarán (debe coincidir con motor_separation
// en pinv_description/urdf/pinv.urdf.xacro).
#define MOTOR_SEPARATION_M  0.75f

// Velocidad del bote [m/s] con ESC_MAX_US en ambos motores. Es lazo
// abierto: v -> us es lineal entre 0 y V_MAX.
// PENDIENTE: calibrar en el lago midiendo con el GPS (Fase 7).
#define V_MAX_MPS           2.0f

// Sin /cmd_vel en este tiempo -> motores a neutro.
#define CMD_VEL_TIMEOUT_MS  500

// -----------------------------------------------------
// GPS RTK ArduSimple simpleRTK2B (u-blox ZED-F9P, ROVER)
//
// Datos por UART1 del simpleRTK2B -> UART1 del ESP32. Las correcciones
// RTCM le llegan por radio a su UART2 (no se toca). IOREF a 3.3 V.
// -----------------------------------------------------
// PENDIENTE: confirmar los pines. GPIO18 NO se puede usar (es el SCL de la IMU).
#define PIN_GPS_RX      16      // RX del ESP32 <- TX1 del simpleRTK2B
#define PIN_GPS_TX      17      // TX del ESP32 -> RX1 del simpleRTK2B
#define GPS_BAUD        38400   // UART1 del ZED-F9P por defecto (si no conecta: 115200)
#define GPS_NAV_HZ      5       // soluciones por segundo (UBX-NAV-PVT)
#define GPS_RETRY_MS    1000    // reintento de gnss.begin()
#define GPS_BEGIN_WAIT_MS 250   // espera máx. de cada begin(); bloquea loop()
#define GPS_TIMEOUT_MS  3000    // sin PVT en este tiempo -> reconectar

// -----------------------------------------------------
// micro-ROS
// -----------------------------------------------------
#define ROS_NODE_NAME           "esp32s3_imu_node"
#define ROS_AGENT_RETRY_MS      2000    // reintento de conexión con el agente
#define ROS_CLOCK_SYNC_MS       30000   // resincronización periódica del reloj
#define ROS_CLOCK_SYNC_TIMEOUT  1000
#define ROS_PUBLISH_FAIL_LIMIT  50      // fallos seguidos (~1 s) = agente perdido
