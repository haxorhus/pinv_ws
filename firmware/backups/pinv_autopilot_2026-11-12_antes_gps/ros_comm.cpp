// =====================================================
// ros_comm.cpp — Entidades micro-ROS, reconexión y reloj
// =====================================================
#include "ros_comm.h"

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>

#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>

#include "esp_timer.h"

#include "config.h"
#include "motors.h"

// -----------------------------------------------------
// Entidades
// -----------------------------------------------------
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

enum RosState { WAITING_AGENT, AGENT_CONNECTED };
static RosState state = WAITING_AGENT;
static uint16_t publishFailStreak = 0;

// -----------------------------------------------------
// Reloj
//
// En esta placa rmw_uros_epoch_nanos() solo avanza de a 1 s entre
// sincronizaciones (se vio en /imu/data: ~50 mensajes con el mismo
// stamp). Justo después de sincronizar sí es exacto, así que se
// guarda ese instante junto con esp_timer_get_time() (us, 64 bits)
// y se interpola:  stamp = epochAtSync + (ahora_us - usAtSync)
// -----------------------------------------------------
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

// -----------------------------------------------------
// /cmd_vel
// -----------------------------------------------------
static void cmdVelCallback(const void *msgIn)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgIn;
    motorsSetCommand((float)msg->linear.x, (float)msg->angular.z);
}

// -----------------------------------------------------
// Ciclo de vida de las entidades
// -----------------------------------------------------
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

// -----------------------------------------------------
// API
// -----------------------------------------------------
void rosInit()
{
    sensor_msgs__msg__Imu__init(&imuMsg);
    rosidl_runtime_c__String__assign(&imuMsg.header.frame_id, "imu_link");

    for (int i = 0; i < 9; i++)
    {
        imuMsg.orientation_covariance[i] = 0.0;
        imuMsg.angular_velocity_covariance[i] = 0.0;
        imuMsg.linear_acceleration_covariance[i] = 0.0;
    }

    float gyroVar[3], accVar[3];
    imuGetMeasuredVariances(gyroVar, accVar);
    for (int i = 0; i < 3; i++)
    {
        imuMsg.angular_velocity_covariance[4 * i] = gyroVar[i];
        imuMsg.linear_acceleration_covariance[4 * i] = accVar[i];
    }

    geometry_msgs__msg__Twist__init(&cmdVelMsg);

    set_microros_transports();
    state = WAITING_AGENT;
}

void rosSpin()
{
    static uint32_t lastAttemptMs = 0;
    const uint32_t now = millis();

    switch (state)
    {
        case WAITING_AGENT:
            if ((uint32_t)(now - lastAttemptMs) >= ROS_AGENT_RETRY_MS)
            {
                lastAttemptMs = now;

                if (createEntities())
                {
                    state = AGENT_CONNECTED;
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

bool rosReady()
{
    return state == AGENT_CONNECTED && clockSynced;
}

void rosPublishImu(const ImuSample &s)
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
        state = WAITING_AGENT;
    }
}
