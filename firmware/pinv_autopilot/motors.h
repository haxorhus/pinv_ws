// =====================================================
// motors.h — Dos ESC diferenciales sin reversa, lazo abierto
//
//   /cmd_vel (v, w) -> vL = v - w*B/2, vR = v + w*B/2
//   recorte a [0, V_MAX] (sin reversa)
//   0 -> ESC_NEUTRAL_US, V_MAX -> ESC_MAX_US (lineal)
//
// Seguridad: neutro durante el armado (ESC_ARM_MS), sin agente
// micro-ROS o sin /cmd_vel por CMD_VEL_TIMEOUT_MS.
// =====================================================
#pragma once

#include <Arduino.h>

// Configura LEDC en los dos pines y deja ambos ESC en neutro.
// Llamar primero en setup(). false si falló la configuración.
bool motorsInit();

// Nuevo comando de velocidad (desde el callback de /cmd_vel).
void motorsSetCommand(float linearX, float angularZ);

// Aplica el comando o neutro según watchdog/armado/conexión.
// Llamar en cada vuelta de loop().
void motorsUpdate(bool agentConnected);

// Fuerza neutro inmediato en ambos ESC.
void motorsStop();

// Último pulso enviado a cada ESC [us] (para depuración).
uint16_t motorsLeftUs();
uint16_t motorsRightUs();
