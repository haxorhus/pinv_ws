// =====================================================
// pinv_autopilot_simple — Firmware de bajo nivel del ASV PINV (ESP32-S3)
// Versión en UN SOLO ARCHIVO (mismo comportamiento que pinv_autopilot/,
// que está separado en config / imu / motors / ros_comm).
//
//  - Lee el ICM-20948, aplica la calibración guardada por
//    calibracion_imu y estima la orientación con Madgwick (50 Hz).
//  - Publica /imu/data (sensor_msgs/Imu, ENU, best-effort) por micro-ROS
//    sobre USB serial, con timestamp sincronizado con el agente.
//  - Escucha /cmd_vel y mueve dos ESC diferenciales sin reversa.
//    Sin agente o sin /cmd_vel por 500 ms, los motores van a neutro.
//
// LED: apagado = esperando agente, encendido = conectado y sincronizado,
//      parpadeo lento = falta la calibración, rápido = falla de LEDC.
//
// Secciones:
//   1. Configuración
//   2. Motores (ESC)
//   3. IMU (ICM-20948 + calibración + Madgwick)
//   4. micro-ROS (entidades, reconexión, reloj)
//   5. setup() / loop()
// =====================================================

#include <Arduino.h>
#include <cmath>

#include <Wire.h>
#include <Preferences.h>
#include "ICM_20948.h"
#include <MadgwickAHRS.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>

#include "esp_timer.h"


// #####################################################
// 1. CONFIGURACIÓN
//    Lo marcado "PENDIENTE" hay que confirmarlo en el bote.
// #####################################################

// --- Pines (ESP32-S3) ---------------------------------
#define PIN_LED         7       // LED de estado
#define PIN_IMU_SDA     8       // I2C ICM-20948
#define PIN_IMU_SCL     18
#define PIN_ESC_LEFT    4       // de Prueba_de_motores (ESC probado)
#define PIN_ESC_RIGHT   5       // PENDIENTE: confirmar con el cableado de Adrian

// --- IMU ICM-20948 ------------------------------------
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

// --- Motores / ESC (datos de Prueba_de_motores) -------
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

// --- micro-ROS ----------------------------------------
#define ROS_NODE_NAME           "esp32s3_imu_node"
#define ROS_AGENT_RETRY_MS      2000    // reintento de conexión con el agente
#define ROS_CLOCK_SYNC_MS       30000   // resincronización periódica del reloj
#define ROS_CLOCK_SYNC_TIMEOUT  1000
#define ROS_PUBLISH_FAIL_LIMIT  50      // fallos seguidos (~1 s) = agente perdido

// --- Constantes ---------------------------------------
static const float G_MS2   = 9.80665f;
static const float DEG2RAD = 0.017453292519943295f;


// #####################################################
// TIPOS (antes de cualquier función, para que los
// prototipos automáticos del IDE de Arduino los vean)
// #####################################################

// Una muestra ya procesada, en el frame del bote (imu_link):
// +X proa, +Y babor, +Z arriba (REP-103).
struct ImuSample
{
    // Orientación en ENU magnético: yaw 0 = este (magnético),
    // antihorario positivo. La declinación se corrige en ROS
    // (navsat.yaml, magnetic_declination_radians).
    float qw, qx, qy, qz;

    float gyroRadS[3];      // [rad/s]
    float accMS2[3];        // [m/s^2], incluye gravedad

    float rpVariance;       // varianza roll/pitch [rad^2]
    float yawVariance;      // varianza yaw [rad^2], crece sin magnetómetro
    bool  magOk;
};

// Calibración (mismo formato que guarda calibracion_imu)
#define CAL_MAGIC   0x494D5531
#define CAL_VERSION 1

struct ImuCalibration
{
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;

    float gyroBiasDps[3];

    float accBiasG[3];
    float accScale[3];

    float magBiasUT[3];
    float magMatrix[9];

    float gyroVariance[3];
    float accVariance[3];
};

enum RosState { WAITING_AGENT, AGENT_CONNECTED };


// #####################################################
// 2. MOTORES — Dos ESC diferenciales sin reversa, lazo abierto
//
//   /cmd_vel (v, w) -> vL = v - w*B/2, vR = v + w*B/2
//   recorte a [0, V_MAX] (sin reversa)
//   0 -> ESC_NEUTRAL_US, V_MAX -> ESC_MAX_US (lineal)
//
// Señal: LEDC a 50 Hz, 14 bits, reloj XTAL (API Arduino-ESP32 3.x).
// Seguridad: neutro durante el armado (ESC_ARM_MS), sin agente
// micro-ROS o sin /cmd_vel por CMD_VEL_TIMEOUT_MS.
// #####################################################

static const uint32_t PWM_MAX = (1UL << ESC_PWM_RES_BITS) - 1;

static float leftMps  = 0.0f;   // velocidad pedida por motor, ya recortada
static float rightMps = 0.0f;
static uint32_t lastCmdMs = 0;
static bool haveCmd = false;

static uint16_t leftUs  = ESC_NEUTRAL_US;   // último pulso enviado [us]
static uint16_t rightUs = ESC_NEUTRAL_US;
static uint16_t leftWrittenUs  = 0;          // último valor escrito al LEDC (0 = nunca)
static uint16_t rightWrittenUs = 0;

// Escribe el ESC solo si el pulso cambió: con el core 3.x, ledcWrite a
// 50 Hz puede esperar al fin del período (hasta 20 ms) y, llamado en cada
// vuelta de loop(), frenaba todo el lazo (IMU publicando a ~10 Hz).
static void writeEsc(uint8_t pin, uint16_t us, uint16_t &writtenUs)
{
    us = constrain(us, ESC_NEUTRAL_US, ESC_MAX_US);
    if (us == writtenUs)
        return;
    writtenUs = us;
    ledcWrite(pin, ((uint64_t)us * PWM_MAX) / ESC_PERIOD_US);
}

// 0..V_MAX [m/s] -> ESC_NEUTRAL_US..ESC_MAX_US
static uint16_t speedToUs(float mps)
{
    const float span = (float)(ESC_MAX_US - ESC_NEUTRAL_US);
    return (uint16_t)(ESC_NEUTRAL_US + span * (mps / V_MAX_MPS) + 0.5f);
}

// Sin reversa: nada por debajo de 0 (también descarta NaN).
static float clampForward(float v)
{
    if (!(v > 0.0f))
        return 0.0f;
    return v > V_MAX_MPS ? V_MAX_MPS : v;
}

// Fuerza neutro inmediato en ambos ESC.
static void motorsStop()
{
    leftMps = rightMps = 0.0f;
    leftUs = rightUs = ESC_NEUTRAL_US;
    writeEsc(PIN_ESC_LEFT, leftUs, leftWrittenUs);
    writeEsc(PIN_ESC_RIGHT, rightUs, rightWrittenUs);
}

// Configura LEDC y deja ambos ESC en neutro. false si falló.
static bool motorsInit()
{
    const bool clockOk = ledcSetClockSource(LEDC_USE_XTAL_CLK);
    const bool leftOk  = ledcAttach(PIN_ESC_LEFT, ESC_PWM_FREQ_HZ, ESC_PWM_RES_BITS);
    const bool rightOk = ledcAttach(PIN_ESC_RIGHT, ESC_PWM_FREQ_HZ, ESC_PWM_RES_BITS);

    motorsStop();

    return clockOk && leftOk && rightOk;
}

// Nuevo comando de velocidad (desde el callback de /cmd_vel).
static void motorsSetCommand(float linearX, float angularZ)
{
    const float halfB = MOTOR_SEPARATION_M / 2.0f;

    leftMps  = clampForward(linearX - angularZ * halfB);
    rightMps = clampForward(linearX + angularZ * halfB);

    lastCmdMs = millis();
    haveCmd = true;
}

// Aplica el comando o neutro según watchdog/armado/conexión.
static void motorsUpdate(bool agentConnected)
{
    const uint32_t now = millis();

    const bool armed = now >= ESC_ARM_MS;
    const bool fresh = haveCmd && (uint32_t)(now - lastCmdMs) <= CMD_VEL_TIMEOUT_MS;

    if (!armed || !agentConnected || !fresh)
    {
        motorsStop();
        return;
    }

    leftUs  = speedToUs(leftMps);
    rightUs = speedToUs(rightMps);
    writeEsc(PIN_ESC_LEFT, leftUs, leftWrittenUs);
    writeEsc(PIN_ESC_RIGHT, rightUs, rightWrittenUs);
}


// #####################################################
// 3. IMU — Lectura del ICM-20948, calibración y Madgwick
// #####################################################

static ImuCalibration cal;
static ICM_20948_I2C icm;
static Madgwick filter;

static const float varRP =
    (IMU_RP_STD_DEG * DEG2RAD) * (IMU_RP_STD_DEG * DEG2RAD);
static const float varYawBase =
    (IMU_YAW_STD_DEG * DEG2RAD) * (IMU_YAW_STD_DEG * DEG2RAD);
static const float YAW_VAR_MAX = (float)(PI * PI);

// Carga la calibración de flash (Preferences "imuCal").
// false si no existe o es de otra versión: correr calibracion_imu.
static bool imuLoadCalibration()
{
    Preferences prefs;
    prefs.begin("imuCal", true);

    if (prefs.getBytesLength("blob") != sizeof(cal))
    {
        prefs.end();
        return false;
    }

    prefs.getBytes("blob", &cal, sizeof(cal));
    prefs.end();

    return cal.magic == CAL_MAGIC && cal.version == CAL_VERSION;
}

// Inicializa I2C y el ICM-20948 (reintenta hasta encontrarlo).
static void imuInit()
{
    Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL);
    Wire.setClock(IMU_I2C_HZ);
    delay(100);

    while (true)
    {
        icm.begin(Wire, IMU_AD0_VAL);
        if (icm.status == ICM_20948_Stat_Ok)
            break;
        delay(1000);
    }

    filter.begin(IMU_UPDATE_HZ);
}

// Lee, calibra y filtra si hay dato nuevo. Llamar a IMU_UPDATE_HZ.
// Devuelve false si el sensor no tenía dato listo.
static bool imuUpdate(ImuSample &out)
{
    if (!icm.dataReady())
        return false;

    icm.getAGMT();

    // --- Calibración, en ejes crudos del chip (así se midió) ---
    const float rawA[3] = { icm.accX() / 1000.0f, icm.accY() / 1000.0f,
                            icm.accZ() / 1000.0f };                 // mg -> g
    const float rawG[3] = { icm.gyrX(), icm.gyrY(), icm.gyrZ() };   // dps
    const float rawM[3] = { icm.magX(), icm.magY(), icm.magZ() };   // uT

    float a[3], g[3], m[3], mb[3];
    for (int i = 0; i < 3; i++)
    {
        a[i]  = (rawA[i] - cal.accBiasG[i]) * cal.accScale[i];
        g[i]  = rawG[i] - cal.gyroBiasDps[i];
        mb[i] = rawM[i] - cal.magBiasUT[i];
    }
    for (int r = 0; r < 3; r++)
        m[r] = cal.magMatrix[3 * r + 0] * mb[0] +
               cal.magMatrix[3 * r + 1] * mb[1] +
               cal.magMatrix[3 * r + 2] * mb[2];

    // --- Magnetómetro a los ejes del acelerómetro/giróscopo ---
#if MAG_AXIS_FIX
    m[1] = -m[1];
    m[2] = -m[2];
#endif

    // --- Ejes del chip -> frame del bote (+X proa, +Y babor, +Z arriba) ---
    // Identidad: el chip está montado alineado con el bote (verificado con
    // check_imu_gravity: gravedad en +Z). Si se monta girado, cambiar acá.

    // --- Validez del magnetómetro ---
    const float field = sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
    out.magOk = std::isfinite(field) && field > MAG_MIN_UT && field < MAG_MAX_UT;

    // --- Covarianza de yaw dinámica ---
    // Sin magnetómetro el yaw se integra del giro: random walk,
    // la varianza crece linealmente con el tiempo.
    const float dt = 1.0f / IMU_UPDATE_HZ;
    static float yawDriftTime = 0.0f;
    yawDriftTime = out.magOk ? 0.0f : yawDriftTime + dt;

    const float yawVar = varYawBase + cal.gyroVariance[2] * dt * yawDriftTime;
    out.yawVariance = yawVar > YAW_VAR_MAX ? YAW_VAR_MAX : yawVar;
    out.rpVariance = varRP;

    // --- Madgwick (gyro en deg/s) ---
    if (out.magOk)
        filter.update(g[0], g[1], g[2], a[0], a[1], a[2], m[0], m[1], m[2]);
    else
        filter.updateIMU(g[0], g[1], g[2], a[0], a[1], a[2]);

    // Madgwick entrega la orientación respecto de un marco con
    // X = norte magnético, Y = oeste, Z = arriba (NWU): yaw 0 mirando
    // al norte. ROS (robot_localization) espera ENU: yaw 0 mirando al
    // este. ENU = rotación de +90 grados en Z sobre NWU:
    //     q_enu = q_z(+90) * q_nwu
    float w, x, y, z;
    filter.getQuaternion(w, x, y, z);

    const float c = 0.70710678f;    // cos(45) = sin(45)
    out.qw = c * (w - z);
    out.qx = c * (x - y);
    out.qy = c * (y + x);
    out.qz = c * (z + w);

    for (int i = 0; i < 3; i++)
    {
        out.gyroRadS[i] = g[i] * DEG2RAD;
        out.accMS2[i]   = a[i] * G_MS2;
    }

    return true;
}


// #####################################################
// 4. micro-ROS — Entidades, reconexión y reloj
//
// Contrato con ROS:
//   /imu/data  sensor_msgs/Imu      publica, best-effort, imu_link, 50 Hz
//   /cmd_vel   geometry_msgs/Twist  suscribe (Nav2 / teleop)
// #####################################################

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_publisher_t imuPublisher;
static rcl_subscription_t cmdVelSubscriber;
static rclc_executor_t executor;

static sensor_msgs__msg__Imu imuMsg;
static geometry_msgs__msg__Twist cmdVelMsg;

// Qué entidades se crearon, para destruir solo esas.
static bool supportOk, nodeOk, pubOk, subOk, executorOk;

static RosState rosState = WAITING_AGENT;
static uint16_t publishFailStreak = 0;

// --- Reloj --------------------------------------------
// En esta placa rmw_uros_epoch_nanos() solo avanza de a 1 s entre
// sincronizaciones (se vio en /imu/data: ~50 mensajes con el mismo
// stamp). Justo después de sincronizar sí es exacto, así que se
// guarda ese instante junto con esp_timer_get_time() (us, 64 bits)
// y se interpola:  stamp = epochAtSync + (ahora_us - usAtSync)
static bool clockSynced = false;
static int64_t epochNsAtSync = 0;
static int64_t localUsAtSync = 0;
static int64_t lastStampNs = 0;
static uint32_t lastSyncMs = 0;

static bool syncClock()
{
    if (rmw_uros_sync_session(ROS_CLOCK_SYNC_TIMEOUT) != RMW_RET_OK)
        return false;

    epochNsAtSync = rmw_uros_epoch_nanos();
    localUsAtSync = esp_timer_get_time();
    lastSyncMs = millis();
    clockSynced = true;
    return true;
}

static int64_t nowNs()
{
    int64_t ns = epochNsAtSync + (esp_timer_get_time() - localUsAtSync) * 1000LL;

    // Una resincronización puede corregir el reloj hacia atrás unos
    // microsegundos; los stamps publicados nunca retroceden.
    if (ns <= lastStampNs)
        ns = lastStampNs + 1000;

    lastStampNs = ns;
    return ns;
}

// --- /cmd_vel -----------------------------------------
static void cmdVelCallback(const void *msgIn)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgIn;
    motorsSetCommand((float)msg->linear.x, (float)msg->angular.z);
}

// --- Ciclo de vida de las entidades -------------------
static void destroyEntities()
{
    if (supportOk)
    {
        rmw_context_t *ctx = rcl_context_get_rmw_context(&support.context);
        (void)rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0);
    }

    if (executorOk) rclc_executor_fini(&executor);
    if (subOk)      (void)rcl_subscription_fini(&cmdVelSubscriber, &node);
    if (pubOk)      (void)rcl_publisher_fini(&imuPublisher, &node);
    if (nodeOk)     (void)rcl_node_fini(&node);
    if (supportOk)  (void)rclc_support_fini(&support);

    supportOk = nodeOk = pubOk = subOk = executorOk = false;
    clockSynced = false;
}

static bool createEntities()
{
    allocator = rcl_get_default_allocator();

    supportOk = rclc_support_init(&support, 0, NULL, &allocator) == RCL_RET_OK;
    if (!supportOk) return false;

    nodeOk = rclc_node_init_default(&node, ROS_NODE_NAME, "", &support) == RCL_RET_OK;
    if (!nodeOk) return false;

    pubOk = rclc_publisher_init_best_effort(
                &imuPublisher, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                "/imu/data") == RCL_RET_OK;
    if (!pubOk) return false;

    subOk = rclc_subscription_init_default(
                &cmdVelSubscriber, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                "/cmd_vel") == RCL_RET_OK;
    if (!subOk) return false;

    executor = rclc_executor_get_zero_initialized_executor();
    executorOk = rclc_executor_init(&executor, &support.context, 1, &allocator) == RCL_RET_OK;
    if (!executorOk) return false;

    return rclc_executor_add_subscription(&executor, &cmdVelSubscriber, &cmdVelMsg,
                                          &cmdVelCallback, ON_NEW_DATA) == RCL_RET_OK;
}

// Configura el transporte serial y el mensaje IMU.
// Llamar después de imuLoadCalibration() (usa sus varianzas).
static void rosInit()
{
    sensor_msgs__msg__Imu__init(&imuMsg);
    rosidl_runtime_c__String__assign(&imuMsg.header.frame_id, "imu_link");

    for (int i = 0; i < 9; i++)
    {
        imuMsg.orientation_covariance[i] = 0.0;
        imuMsg.angular_velocity_covariance[i] = 0.0;
        imuMsg.linear_acceleration_covariance[i] = 0.0;
    }

    // Varianzas medidas en la calibración (diagonal).
    for (int i = 0; i < 3; i++)
    {
        imuMsg.angular_velocity_covariance[4 * i] = cal.gyroVariance[i];
        imuMsg.linear_acceleration_covariance[4 * i] = cal.accVariance[i];
    }

    geometry_msgs__msg__Twist__init(&cmdVelMsg);

    set_microros_transports();
    rosState = WAITING_AGENT;
}

// true con el agente conectado y el reloj sincronizado.
static bool rosReady()
{
    return rosState == AGENT_CONNECTED && clockSynced;
}

// Máquina de estados: conecta/reconecta con el agente, resincroniza
// el reloj y procesa /cmd_vel. Llamar en cada vuelta de loop().
static void rosSpin()
{
    static uint32_t lastAttemptMs = 0;
    const uint32_t now = millis();

    switch (rosState)
    {
        case WAITING_AGENT:
            if ((uint32_t)(now - lastAttemptMs) >= ROS_AGENT_RETRY_MS)
            {
                lastAttemptMs = now;

                if (createEntities())
                {
                    rosState = AGENT_CONNECTED;
                    publishFailStreak = 0;
                    syncClock();
                }
                else
                {
                    destroyEntities();
                }
            }
            break;

        case AGENT_CONNECTED:
            // Sin reloj no se publica: reintentar cada segundo. Con reloj,
            // resincronizar periódicamente para corregir la deriva.
            if (!clockSynced)
            {
                if ((uint32_t)(now - lastSyncMs) >= 1000)
                {
                    lastSyncMs = now;
                    syncClock();
                }
            }
            else if ((uint32_t)(now - lastSyncMs) >= ROS_CLOCK_SYNC_MS)
            {
                syncClock();
            }

            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
            break;
    }
}

// Publica una muestra con timestamp sincronizado con el agente.
static void rosPublishImu(const ImuSample &s)
{
    if (!rosReady())
        return;

    const int64_t ns = nowNs();
    imuMsg.header.stamp.sec = (int32_t)(ns / 1000000000LL);
    imuMsg.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);

    imuMsg.orientation.w = s.qw;
    imuMsg.orientation.x = s.qx;
    imuMsg.orientation.y = s.qy;
    imuMsg.orientation.z = s.qz;

    imuMsg.orientation_covariance[0] = s.rpVariance;
    imuMsg.orientation_covariance[4] = s.rpVariance;
    imuMsg.orientation_covariance[8] = s.yawVariance;

    imuMsg.angular_velocity.x = s.gyroRadS[0];
    imuMsg.angular_velocity.y = s.gyroRadS[1];
    imuMsg.angular_velocity.z = s.gyroRadS[2];

    imuMsg.linear_acceleration.x = s.accMS2[0];
    imuMsg.linear_acceleration.y = s.accMS2[1];
    imuMsg.linear_acceleration.z = s.accMS2[2];

    if (rcl_publish(&imuPublisher, &imuMsg, NULL) == RCL_RET_OK)
    {
        publishFailStreak = 0;
        return;
    }

    // Detección de desconexión por fallos seguidos de publicación (no por
    // ping: sobre serial el ping XRCE no siempre se responde).
    if (++publishFailStreak >= ROS_PUBLISH_FAIL_LIMIT)
    {
        destroyEntities();
        rosState = WAITING_AGENT;
    }
}


// #####################################################
// 5. SETUP / LOOP
// #####################################################

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
        blinkForever(100);      // parpadeo rápido: falla de LEDC

    if (!imuLoadCalibration())
        blinkForever(500);      // parpadeo lento: falta calibración

    imuInit();
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

    motorsUpdate(rosReady());

    digitalWrite(PIN_LED, rosReady());

    delay(1);
}
