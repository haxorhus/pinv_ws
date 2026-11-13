// =====================================================
// imu.h — Lectura del ICM-20948, calibración y Madgwick
// =====================================================
#pragma once

#include <Arduino.h>

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

// Carga la calibración de flash (Preferences "imuCal").
// false si no existe o es de otra versión: correr calibracion_imu.
bool imuLoadCalibration();

// Inicializa I2C y el ICM-20948 (reintenta hasta encontrarlo).
void imuInit();

// Lee, calibra y filtra si hay dato nuevo. Llamar a IMU_UPDATE_HZ.
// Devuelve false si el sensor no tenía dato listo.
bool imuUpdate(ImuSample &out);

// Varianzas medidas en la calibración (para las covarianzas del mensaje).
void imuGetMeasuredVariances(float gyroVar[3], float accVar[3]);
