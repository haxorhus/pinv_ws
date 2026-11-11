// =====================================================
// imu.cpp — Lectura del ICM-20948, calibración y Madgwick
// =====================================================
#include "imu.h"

#include <cmath>

#include <Wire.h>
#include <Preferences.h>
#include "ICM_20948.h"
#include <MadgwickAHRS.h>

#include "config.h"

static const float G_MS2   = 9.80665f;
static const float DEG2RAD = 0.017453292519943295f;

// -----------------------------------------------------
// Calibración (mismo formato que guarda calibracion_imu)
// -----------------------------------------------------
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

static ImuCalibration cal;
static ICM_20948_I2C icm;
static Madgwick filter;

static const float varRP =
    (IMU_RP_STD_DEG * DEG2RAD) * (IMU_RP_STD_DEG * DEG2RAD);
static const float varYawBase =
    (IMU_YAW_STD_DEG * DEG2RAD) * (IMU_YAW_STD_DEG * DEG2RAD);
static const float YAW_VAR_MAX = (float)(PI * PI);

bool imuLoadCalibration()
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

void imuInit()
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

void imuGetMeasuredVariances(float gyroVar[3], float accVar[3])
{
    for (int i = 0; i < 3; i++)
    {
        gyroVar[i] = cal.gyroVariance[i];
        accVar[i]  = cal.accVariance[i];
    }
}

// Calibración en ejes crudos del chip (así se midió).
static void applyCalibration(const float rawAcc[3], const float rawGyro[3],
                             const float rawMag[3],
                             float acc[3], float gyro[3], float mag[3])
{
    for (int i = 0; i < 3; i++)
    {
        acc[i]  = (rawAcc[i] - cal.accBiasG[i]) * cal.accScale[i];
        gyro[i] = rawGyro[i] - cal.gyroBiasDps[i];
    }

    float m[3];
    for (int i = 0; i < 3; i++)
        m[i] = rawMag[i] - cal.magBiasUT[i];

    for (int r = 0; r < 3; r++)
        mag[r] = cal.magMatrix[3 * r + 0] * m[0] +
                 cal.magMatrix[3 * r + 1] * m[1] +
                 cal.magMatrix[3 * r + 2] * m[2];
}

// Lleva el magnetómetro a los ejes del acelerómetro/giróscopo.
static void magToAccelAxes(float mag[3])
{
#if MAG_AXIS_FIX
    mag[1] = -mag[1];
    mag[2] = -mag[2];
#endif
}

// Ejes del chip -> frame del bote (+X proa, +Y babor, +Z arriba).
// Identidad: el chip está montado alineado con el bote (verificado con
// check_imu_gravity: gravedad en +Z). Si se monta girado, cambiar acá.
static void chipToBoat(float v[3])
{
    (void)v;
}

bool imuUpdate(ImuSample &out)
{
    if (!icm.dataReady())
        return false;

    icm.getAGMT();

    const float rawA[3] = { icm.accX() / 1000.0f, icm.accY() / 1000.0f,
                            icm.accZ() / 1000.0f };     // mg -> g
    const float rawG[3] = { icm.gyrX(), icm.gyrY(), icm.gyrZ() };   // dps
    const float rawM[3] = { icm.magX(), icm.magY(), icm.magZ() };   // uT

    float a[3], g[3], m[3];
    applyCalibration(rawA, rawG, rawM, a, g, m);
    magToAccelAxes(m);

    chipToBoat(a);
    chipToBoat(g);
    chipToBoat(m);

    // --- Validez del magnetómetro -----------------------
    const float field = sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
    out.magOk = std::isfinite(field) && field > MAG_MIN_UT && field < MAG_MAX_UT;

    // --- Covarianza de yaw dinámica ---------------------
    // Sin magnetómetro el yaw se integra del giro: random walk,
    // la varianza crece linealmente con el tiempo.
    const float dt = 1.0f / IMU_UPDATE_HZ;
    static float yawDriftTime = 0.0f;
    yawDriftTime = out.magOk ? 0.0f : yawDriftTime + dt;

    float yawVar = varYawBase + cal.gyroVariance[2] * dt * yawDriftTime;
    out.yawVariance = yawVar > YAW_VAR_MAX ? YAW_VAR_MAX : yawVar;
    out.rpVariance = varRP;

    // --- Madgwick (gyro en deg/s) -----------------------
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
