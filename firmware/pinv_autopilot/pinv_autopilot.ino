
#include <Wire.h>
#include "ICM_20948.h"
#include <Preferences.h>
#include <MadgwickAHRS.h>

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>

#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>

#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>

// =====================================================
// Hardware
// =====================================================

#define SDA_PIN 8
#define SCL_PIN 18
#define AD0_VAL 0

#define G_MS2 9.80665f
#define DEG2RAD 0.017453292519943295f

#define UPDATE_HZ 50
#define UPDATE_US 20000

const int Led = 7;

ICM_20948_I2C imu;
Madgwick filter;
Preferences prefs;


// =====================================================
// Calibración
// =====================================================

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

ImuCalibration cal;


// =====================================================
// micro-ROS
// =====================================================

rcl_node_t node;
rcl_publisher_t imuPublisher;
rclc_support_t support;
rcl_allocator_t allocator;

sensor_msgs__msg__Imu imuMsg;

rcl_subscription_t cmdVelSubscriber;
rclc_executor_t executor;

geometry_msgs__msg__Twist cmdVelMsg;


// =====================================================
// cmd_vel (Nav2)
// =====================================================
//
// Nav2 publica geometry_msgs/Twist en /cmd_vel:
//   linear.x  -> avance [m/s]
//   angular.z -> giro   [rad/s]
//
// Se convierte a velocidad de cada motor con
// cinemática diferencial:
//   vL = v - w * B/2
//   vR = v + w * B/2
// =====================================================

// Separación entre motores [m] (AJUSTAR al ASV)
#define WHEEL_SEPARATION 0.40f

// Si no llega cmd_vel en este tiempo, se detienen
// los motores (seguridad ante pérdida de Nav2).
#define CMD_VEL_TIMEOUT_MS 500

volatile float cmdLinearX  = 0.0f;
volatile float cmdAngularZ = 0.0f;

volatile float motorLeftSpeed  = 0.0f;   // [m/s]
volatile float motorRightSpeed = 0.0f;   // [m/s]

uint32_t lastCmdVelMs = 0;

enum MicroRosState
{
    WAITING_AGENT,
    AGENT_CONNECTED
};

MicroRosState rosState = WAITING_AGENT;

// Desconexión detectada por fallos consecutivos de
// rcl_publish (no por ping: sobre transporte serial el
// ping XRCE queda fuera de sesión y el agente no
// siempre lo responde, lo que generaba ciclos falsos de
// conexión/desconexión).
#define PUBLISH_FAIL_LIMIT 50

uint16_t publishFailStreak = 0;


// =====================================================
// Covarianza de orientación
// =====================================================
//
// roll/pitch: siempre corregidos por el acelerómetro,
// se mantienen constantes.
//
// yaw: cuando el magnetómetro es válido (magOK) se usa
// varYawBase. Cuando no, el yaw solo se obtiene por
// integración del giro y su incertidumbre crece con el
// tiempo, por lo que se infla dinámicamente en publishIMU().
// =====================================================

const float varRP =
    (3.0f * DEG2RAD) * (3.0f * DEG2RAD);

const float varYawBase =
    (10.0f * DEG2RAD) * (10.0f * DEG2RAD);

const float YAW_VAR_MAX =
    (float)(PI * PI);


// =====================================================
// Error
// =====================================================

void errorLoop()
{
    while (1)
    {
        digitalWrite(Led,!digitalRead(Led));
        delay(500);
    };
}

// Usado solo dentro de funciones que retornan bool
// (createEntities), para poder reintentar sin bloquear
// el equipo ante una desconexión del agente.
#define RCCHECK(fn)                  \
{                                    \
    rcl_ret_t rc = fn;               \
    if (rc != RCL_RET_OK)            \
        return false;                 \
}

// Ejecuta X cada MS milisegundos sin bloquear el loop()
#define EXECUTE_EVERY_N_MS(MS, X)                        \
    do                                                    \
    {                                                      \
        static uint32_t lastExec_ = 0;                      \
        uint32_t nowMs_ = millis();                          \
        if ((uint32_t)(nowMs_ - lastExec_) >= (MS))           \
        {                                                      \
            X;                                                  \
            lastExec_ = nowMs_;                                   \
        }                                                          \
    } while (0)


// =====================================================
// Cargar calibración
// =====================================================

bool loadCalibration()
{
    prefs.begin("imuCal", true);

    size_t len =
        prefs.getBytesLength("blob");

    if (len != sizeof(cal))
    {
        prefs.end();
        return false;
    }

    prefs.getBytes(
        "blob",
        &cal,
        sizeof(cal)
    );

    prefs.end();

    if (cal.magic != CAL_MAGIC)
        return false;

    if (cal.version != CAL_VERSION)
        return false;

    return true;
}


// =====================================================
// Inicializar IMU
// =====================================================

void initIMU()
{
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    delay(100);

    while (true)
    {
        imu.begin(Wire, AD0_VAL);

        if (imu.status == ICM_20948_Stat_Ok)
            break;

        delay(1000);
    }
}


// =====================================================
// Aplicar calibración
// =====================================================

void applyCalibration(
    float rawAcc[3],
    float rawGyro[3],
    float rawMag[3],
    float acc[3],
    float gyro[3],
    float mag[3])
{
    for (int i = 0; i < 3; i++)
    {
        acc[i] =
            (rawAcc[i] - cal.accBiasG[i])
            * cal.accScale[i];

        gyro[i] =
            rawGyro[i] - cal.gyroBiasDps[i];
    }

    float m[3];

    for (int i = 0; i < 3; i++)
        m[i] =
            rawMag[i] - cal.magBiasUT[i];

    mag[0] =
        cal.magMatrix[0] * m[0] +
        cal.magMatrix[1] * m[1] +
        cal.magMatrix[2] * m[2];

    mag[1] =
        cal.magMatrix[3] * m[0] +
        cal.magMatrix[4] * m[1] +
        cal.magMatrix[5] * m[2];

    mag[2] =
        cal.magMatrix[6] * m[0] +
        cal.magMatrix[7] * m[1] +
        cal.magMatrix[8] * m[2];
}


// =====================================================
// Transformación de ejes
//
// IMPORTANTE:
//
//
// +X = frente del ASV
// +Y = izquierda
// +Z = arriba
//
// =====================================================

void sensorToRos(
    float &x,
    float &y,
    float &z)
{
    // Identidad por ahora

    float X = x;
    float Y = y;
    float Z = z;

    x = X;
    y = Y;
    z = Z;
}


// =====================================================
// Leer, filtrar y publicar
// =====================================================

// Retorna false solo si se intentó publicar y
// rcl_publish falló (usado para detectar que el
// agente se desconectó). Si no había dato nuevo no
// cuenta como falla.
bool publishIMU()
{
    if (!imu.dataReady())
        return true;

    imu.getAGMT();

    // -----------------------------------------------
    // Valores originales
    // -----------------------------------------------

    float rawA[3] =
    {
        imu.accX() / 1000.0f,
        imu.accY() / 1000.0f,
        imu.accZ() / 1000.0f
    };

    float rawG[3] =
    {
        imu.gyrX(),
        imu.gyrY(),
        imu.gyrZ()
    };

    float rawM[3] =
    {
        imu.magX(),
        imu.magY(),
        imu.magZ()
    };


    // -----------------------------------------------
    // Calibración
    // -----------------------------------------------

    float a[3];
    float g[3];
    float m[3];

    applyCalibration(
        rawA,
        rawG,
        rawM,
        a,
        g,
        m
    );


    // -----------------------------------------------
    // Transformación de ejes
    // -----------------------------------------------

    sensorToRos(a[0], a[1], a[2]);
    sensorToRos(g[0], g[1], g[2]);
    sensorToRos(m[0], m[1], m[2]);


    // -----------------------------------------------
    // Verificar magnetómetro
    // -----------------------------------------------

    float fieldMagnitude =
        sqrtf(
            m[0] * m[0] +
            m[1] * m[1] +
            m[2] * m[2]
        );

    bool magOK =
        isfinite(fieldMagnitude) &&
        fieldMagnitude > 10.0f &&
        fieldMagnitude < 100.0f;


    // -----------------------------------------------
    // Covarianza de yaw dinámica
    //
    // Sin magnetómetro válido el yaw solo se integra del
    // giro. Para ruido blanco integrado, la varianza de
    // un "random walk" crece LINEALMENTE con el tiempo:
    // Var(t) = sigma^2 * dt * t.
    // -----------------------------------------------

    const float dt = 1.0f / UPDATE_HZ;

    static float yawDriftTime = 0.0f;

    if (magOK)
        yawDriftTime = 0.0f;
    else
        yawDriftTime += dt;

    float yawVar =
        varYawBase +
        cal.gyroVariance[2] * dt * yawDriftTime;

    if (yawVar > YAW_VAR_MAX)
        yawVar = YAW_VAR_MAX;

    imuMsg.orientation_covariance[8] = yawVar;


    // -----------------------------------------------
    // Madgwick
    //
    // MadgwickAHRS espera gyro en deg/s.
    // -----------------------------------------------

    if (magOK)
    {
        filter.update(
            g[0], g[1], g[2],
            a[0], a[1], a[2],
            m[0], m[1], m[2]
        );
    }
    else
    {
        // Si existe una perturbación magnética fuerte,
        // trabajar temporalmente como 6-DOF.
        filter.updateIMU(
            g[0], g[1], g[2],
            a[0], a[1], a[2]
        );
    }


    // -----------------------------------------------
    // Quaternion
    //
    // Se lee directamente del filtro (sin pasar por
    // Euler) para evitar el costo trigonométrico y la
    // pérdida de precisión cerca de gimbal lock.
    // -----------------------------------------------

    float qw, qx, qy, qz;

    filter.getQuaternion(qw, qx, qy, qz);

    imuMsg.orientation.x = qx;
    imuMsg.orientation.y = qy;
    imuMsg.orientation.z = qz;
    imuMsg.orientation.w = qw;


    // -----------------------------------------------
    // Giro ROS -> rad/s
    // -----------------------------------------------

    imuMsg.angular_velocity.x =
        g[0] * DEG2RAD;

    imuMsg.angular_velocity.y =
        g[1] * DEG2RAD;

    imuMsg.angular_velocity.z =
        g[2] * DEG2RAD;


    // -----------------------------------------------
    // Aceleración ROS -> m/s²
    // -----------------------------------------------

    imuMsg.linear_acceleration.x =
        a[0] * G_MS2;

    imuMsg.linear_acceleration.y =
        a[1] * G_MS2;

    imuMsg.linear_acceleration.z =
        a[2] * G_MS2;


    // -----------------------------------------------
    // Timestamp ROS
    // -----------------------------------------------

    if (rmw_uros_epoch_synchronized())
    {
        int64_t ns =
            rmw_uros_epoch_nanos();

        imuMsg.header.stamp.sec =
            ns / 1000000000LL;

        imuMsg.header.stamp.nanosec =
            ns % 1000000000LL;
    }


    // -----------------------------------------------
    // Publicar
    // -----------------------------------------------

    rcl_ret_t rc =
        rcl_publish(
            &imuPublisher,
            &imuMsg,
            NULL
        );

    return rc == RCL_RET_OK;
}


// =====================================================
// Callback cmd_vel
// =====================================================

void cmdVelCallback(const void *msgIn)
{
    const geometry_msgs__msg__Twist *msg =
        (const geometry_msgs__msg__Twist *)msgIn;

    cmdLinearX  = msg->linear.x;
    cmdAngularZ = msg->angular.z;

    motorLeftSpeed =
        cmdLinearX - cmdAngularZ * WHEEL_SEPARATION / 2.0f;

    motorRightSpeed =
        cmdLinearX + cmdAngularZ * WHEEL_SEPARATION / 2.0f;

    lastCmdVelMs = millis();
}

void stopMotors()
{
    cmdLinearX      = 0.0f;
    cmdAngularZ     = 0.0f;
    motorLeftSpeed  = 0.0f;
    motorRightSpeed = 0.0f;
}

// Aquí se aplican motorLeftSpeed / motorRightSpeed a
// los drivers (PWM/ESC). Por ahora solo watchdog.
void updateMotors()
{
    if ((uint32_t)(millis() - lastCmdVelMs)
        > CMD_VEL_TIMEOUT_MS)
    {
        stopMotors();
    }

    // TODO: convertir motorLeftSpeed / motorRightSpeed
    // a PWM/ESC según el hardware del ASV.
}


// =====================================================
// Mensaje IMU (una sola vez, independiente del
// ciclo de conexión/reconexión con el agente)
// =====================================================

void initImuMessage()
{
    sensor_msgs__msg__Imu__init(
        &imuMsg
    );

    rosidl_runtime_c__String__assign(
        &imuMsg.header.frame_id,
        "imu_link"
    );

    for (int i = 0; i < 9; i++)
    {
        imuMsg.orientation_covariance[i] = 0.0;

        imuMsg.angular_velocity_covariance[i] = 0.0;

        imuMsg.linear_acceleration_covariance[i] = 0.0;
    }

    // Valores conservadores iniciales (roll/pitch
    // constantes; yaw se recalcula en publishIMU()).

    imuMsg.orientation_covariance[0] =
        varRP;

    imuMsg.orientation_covariance[4] =
        varRP;

    imuMsg.orientation_covariance[8] =
        varYawBase;


    // Covarianzas medidas durante calibración

    imuMsg.angular_velocity_covariance[0] =
        cal.gyroVariance[0];

    imuMsg.angular_velocity_covariance[4] =
        cal.gyroVariance[1];

    imuMsg.angular_velocity_covariance[8] =
        cal.gyroVariance[2];


    imuMsg.linear_acceleration_covariance[0] =
        cal.accVariance[0];

    imuMsg.linear_acceleration_covariance[4] =
        cal.accVariance[1];

    imuMsg.linear_acceleration_covariance[8] =
        cal.accVariance[2];
}


// =====================================================
// Ciclo de vida de las entidades micro-ROS
//
// Separado de setup() para poder reintentar la
// creación y destruir limpiamente ante una
// desconexión del agente, sin reiniciar el equipo.
// =====================================================

bool createEntities()
{
    allocator =
        rcl_get_default_allocator();

    RCCHECK(
        rclc_support_init(
            &support,
            0,
            NULL,
            &allocator
        )
    );

    RCCHECK(
        rclc_node_init_default(
            &node,
            "esp32s3_imu_node",
            "",
            &support
        )
    );

    RCCHECK(
        rclc_publisher_init_best_effort(
            &imuPublisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                sensor_msgs,
                msg,
                Imu
            ),
            "/imu/data"
        )
    );

    RCCHECK(
        rclc_subscription_init_default(
            &cmdVelSubscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                geometry_msgs,
                msg,
                Twist
            ),
            "/cmd_vel"
        )
    );

    executor = rclc_executor_get_zero_initialized_executor();

    RCCHECK(
        rclc_executor_init(
            &executor,
            &support.context,
            1,
            &allocator
        )
    );

    RCCHECK(
        rclc_executor_add_subscription(
            &executor,
            &cmdVelSubscriber,
            &cmdVelMsg,
            &cmdVelCallback,
            ON_NEW_DATA
        )
    );

    return true;
}

void destroyEntities()
{
    rmw_context_t *rmwContext =
        rcl_context_get_rmw_context(&support.context);

    (void)rmw_uros_set_context_entity_destroy_session_timeout(
        rmwContext,
        0
    );

    rclc_executor_fini(&executor);
    rcl_subscription_fini(&cmdVelSubscriber, &node);
    rcl_publisher_fini(&imuPublisher, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
    pinMode(Led, OUTPUT);
    digitalWrite(Led, LOW);

    // La calibración debe existir
    if (!loadCalibration())
        errorLoop();

    initIMU();

    // Madgwick a 50 Hz
    filter.begin(UPDATE_HZ);

    initImuMessage();

    geometry_msgs__msg__Twist__init(&cmdVelMsg);

    set_microros_transports();

    rosState = WAITING_AGENT;
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
    switch (rosState)
    {
        case WAITING_AGENT:

            EXECUTE_EVERY_N_MS(2000,
                if (createEntities())
                {
                    rosState = AGENT_CONNECTED;

                    publishFailStreak = 0;

                    rmw_uros_sync_session(1000);
                }
                else
                {
                    destroyEntities();
                }
            );

            break;

        case AGENT_CONNECTED:
        {
            // Resincronizar reloj periódicamente
            // (evita deriva en misiones largas).
            EXECUTE_EVERY_N_MS(30000,
                rmw_uros_sync_session(1000);
            );

            // Procesar cmd_vel entrante (no bloqueante)
            rclc_executor_spin_some(
                &executor,
                RCL_MS_TO_NS(1)
            );

            static uint32_t previous = 0;

            uint32_t now = micros();

            if ((uint32_t)(now - previous)
                >= UPDATE_US)
            {
                previous = now;

                if (publishIMU())
                    publishFailStreak = 0;
                else
                    publishFailStreak++;

                if (publishFailStreak >= PUBLISH_FAIL_LIMIT)
                {
                    destroyEntities();

                    rosState = WAITING_AGENT;
                }
            }

            break;
        }
    }

    // Watchdog de motores: sin agente o sin cmd_vel
    // reciente, los motores quedan en 0.
    updateMotors();

    // LED encendido solo con el agente conectado
    digitalWrite(Led, rosState == AGENT_CONNECTED);

    delay(1);
}