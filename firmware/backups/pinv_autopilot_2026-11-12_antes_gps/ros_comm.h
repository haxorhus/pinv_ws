// =====================================================
// ros_comm.h — Entidades micro-ROS, reconexión y reloj
//
// Contrato con ROS:
//   /imu/data  sensor_msgs/Imu      publica, best-effort, imu_link, 50 Hz
//   /cmd_vel   geometry_msgs/Twist  suscribe (Nav2 / teleop)
// =====================================================
#pragma once

#include <Arduino.h>
#include "imu.h"

// Configura el transporte serial y el mensaje IMU. Llamar en setup()
// después de imuLoadCalibration().
void rosInit();

// Máquina de estados: conecta/reconecta con el agente, resincroniza
// el reloj y procesa /cmd_vel. Llamar en cada vuelta de loop().
void rosSpin();

// true con el agente conectado y el reloj sincronizado.
bool rosReady();

// Publica una muestra con timestamp sincronizado con el agente.
// Si no está listo, no publica.
void rosPublishImu(const ImuSample &s);
