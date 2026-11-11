// =====================================================
// motors.cpp — Dos ESC diferenciales sin reversa, lazo abierto
//
// Señal: LEDC a 50 Hz, 14 bits, reloj XTAL (igual que
// Prueba_de_motores, API del core Arduino-ESP32 3.x).
// =====================================================
#include "motors.h"

#include "config.h"

static const uint32_t PWM_MAX = (1UL << ESC_PWM_RES_BITS) - 1;

static float leftMps  = 0.0f;   // velocidad pedida por motor, ya recortada
static float rightMps = 0.0f;
static uint32_t lastCmdMs = 0;
static bool haveCmd = false;

static uint16_t leftUs  = ESC_NEUTRAL_US;
static uint16_t rightUs = ESC_NEUTRAL_US;

static uint32_t usToDuty(uint16_t us)
{
    return ((uint64_t)us * PWM_MAX) / ESC_PERIOD_US;
}

static void writeEsc(uint8_t pin, uint16_t us)
{
    us = constrain(us, ESC_NEUTRAL_US, ESC_MAX_US);
    ledcWrite(pin, usToDuty(us));
}

// 0..V_MAX [m/s] -> ESC_NEUTRAL_US..ESC_MAX_US
static uint16_t speedToUs(float mps)
{
    const float span = (float)(ESC_MAX_US - ESC_NEUTRAL_US);
    return (uint16_t)(ESC_NEUTRAL_US + span * (mps / V_MAX_MPS) + 0.5f);
}

static float clampForward(float v)
{
    // Sin reversa: nada por debajo de 0.
    if (!(v > 0.0f))        // también descarta NaN
        return 0.0f;
    return v > V_MAX_MPS ? V_MAX_MPS : v;
}

bool motorsInit()
{
    const bool clockOk = ledcSetClockSource(LEDC_USE_XTAL_CLK);
    const bool leftOk  = ledcAttach(PIN_ESC_LEFT, ESC_PWM_FREQ_HZ, ESC_PWM_RES_BITS);
    const bool rightOk = ledcAttach(PIN_ESC_RIGHT, ESC_PWM_FREQ_HZ, ESC_PWM_RES_BITS);

    motorsStop();

    return clockOk && leftOk && rightOk;
}

void motorsSetCommand(float linearX, float angularZ)
{
    const float halfB = MOTOR_SEPARATION_M / 2.0f;

    leftMps  = clampForward(linearX - angularZ * halfB);
    rightMps = clampForward(linearX + angularZ * halfB);

    lastCmdMs = millis();
    haveCmd = true;
}

void motorsStop()
{
    leftMps = rightMps = 0.0f;
    leftUs = rightUs = ESC_NEUTRAL_US;
    writeEsc(PIN_ESC_LEFT, leftUs);
    writeEsc(PIN_ESC_RIGHT, rightUs);
}

void motorsUpdate(bool agentConnected)
{
    const uint32_t now = millis();

    const bool armed   = now >= ESC_ARM_MS;
    const bool fresh   = haveCmd && (uint32_t)(now - lastCmdMs) <= CMD_VEL_TIMEOUT_MS;

    if (!armed || !agentConnected || !fresh)
    {
        motorsStop();
        return;
    }

    leftUs  = speedToUs(leftMps);
    rightUs = speedToUs(rightMps);
    writeEsc(PIN_ESC_LEFT, leftUs);
    writeEsc(PIN_ESC_RIGHT, rightUs);
}

uint16_t motorsLeftUs()  { return leftUs; }
uint16_t motorsRightUs() { return rightUs; }
