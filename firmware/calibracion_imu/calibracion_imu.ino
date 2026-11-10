#include <Wire.h>
#include "ICM_20948.h"
#include <Preferences.h>

#define SDA_PIN 8
#define SCL_PIN 18
#define AD0_VAL 0

#define G_MS2     9.80665f
#define DEG2RAD   0.017453292519943295f

ICM_20948_I2C imu;
Preferences prefs;

// =====================================================
// Estructura de calibración
// =====================================================

#define CAL_MAGIC   0x494D5531
#define CAL_VERSION 1

struct ImuCalibration
{
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;

    // Gyro bias [deg/s]
    float gyroBiasDps[3];

    // Accel:
    // corrected = (raw_g - bias) * scale
    float accBiasG[3];
    float accScale[3];

    // Magnetómetro:
    // corrected = M * (raw - bias)
    float magBiasUT[3];
    float magMatrix[9];

    // Varianzas para sensor_msgs/Imu
    // gyro [(rad/s)^2]
    float gyroVariance[3];

    // accel [(m/s^2)^2]
    float accVariance[3];
};

ImuCalibration cal;


// =====================================================
// Estadística incremental
// =====================================================

struct RunningStats
{
    uint32_t n = 0;
    double mean = 0.0;
    double M2 = 0.0;

    void add(double x)
    {
        n++;

        double delta = x - mean;
        mean += delta / n;

        double delta2 = x - mean;
        M2 += delta * delta2;
    }

    double variance()
    {
        if (n < 2)
            return 0.0;

        return M2 / (n - 1);
    }
};


// =====================================================
// Esperar ENTER
// =====================================================

void waitEnter(const char *mensaje)
{
    Serial.println();
    Serial.println(mensaje);
    Serial.println("Presiona ENTER cuando estes listo.");

    while (Serial.available())
        Serial.read();

    while (!Serial.available())
        delay(10);

    while (Serial.available())
        Serial.read();

    delay(500);
}


// =====================================================
// Inicializar IMU
// =====================================================

void initIMU()
{
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);

    delay(100);

    while (true)
    {
        imu.begin(Wire, AD0_VAL);

        if (imu.status == ICM_20948_Stat_Ok)
            break;

        Serial.print("Error IMU: ");
        Serial.println(imu.statusString());

        delay(1000);
    }

    Serial.println("ICM-20948 inicializado.");
}


// =====================================================
// Obtener una muestra
// =====================================================

bool getSample(float acc[3], float gyr[3], float mag[3])
{
    uint32_t inicio = millis();

    while (!imu.dataReady())
    {
        if ((millis() - inicio) > 200)
            return false;

        delay(1);
    }

    imu.getAGMT();

    // mg -> g
    acc[0] = imu.accX() / 1000.0f;
    acc[1] = imu.accY() / 1000.0f;
    acc[2] = imu.accZ() / 1000.0f;

    // deg/s
    gyr[0] = imu.gyrX();
    gyr[1] = imu.gyrY();
    gyr[2] = imu.gyrZ();

    // uT
    mag[0] = imu.magX();
    mag[1] = imu.magY();
    mag[2] = imu.magZ();

    return true;
}


// =====================================================
// Promediar varias muestras
// =====================================================

void averageSamples(int cantidad,
                    float accMean[3],
                    float gyrMean[3])
{
    double accSum[3] = {0, 0, 0};
    double gyrSum[3] = {0, 0, 0};

    int validas = 0;

    while (validas < cantidad)
    {
        float a[3];
        float g[3];
        float m[3];

        if (getSample(a, g, m))
        {
            for (int i = 0; i < 3; i++)
            {
                accSum[i] += a[i];
                gyrSum[i] += g[i];
            }

            validas++;
        }

        delay(2);
    }

    for (int i = 0; i < 3; i++)
    {
        accMean[i] = accSum[i] / cantidad;
        gyrMean[i] = gyrSum[i] / cantidad;
    }
}


// =====================================================
// Calibrar giroscopio
// =====================================================

void calibrateGyro()
{
    waitEnter(
        "CALIBRACION GIROSCOPIO\n"
        "Deja el IMU COMPLETAMENTE quieto sobre una superficie."
    );

    Serial.println("Midiendo gyro...");

    float acc[3];
    float gyr[3];

    averageSamples(2000, acc, gyr);

    for (int i = 0; i < 3; i++)
        cal.gyroBiasDps[i] = gyr[i];

    Serial.println("Bias gyro [deg/s]:");

    Serial.print("X = ");
    Serial.println(cal.gyroBiasDps[0], 6);

    Serial.print("Y = ");
    Serial.println(cal.gyroBiasDps[1], 6);

    Serial.print("Z = ");
    Serial.println(cal.gyroBiasDps[2], 6);
}


// =====================================================
// Capturar una cara para acelerómetro
// =====================================================

float captureAccelAxis(
    const char *mensaje,
    int eje)
{
    waitEnter(mensaje);

    float acc[3];
    float gyr[3];

    averageSamples(800, acc, gyr);

    Serial.print("Valor obtenido = ");
    Serial.println(acc[eje], 6);

    return acc[eje];
}


// =====================================================
// Calibración acelerómetro 6 caras
// =====================================================

void calibrateAccel()
{
    Serial.println();
    Serial.println("==============================");
    Serial.println(" CALIBRACION ACELEROMETRO");
    Serial.println("==============================");

    float positive[3];
    float negative[3];

    positive[0] = captureAccelAxis(
        "Coloca el sensor con +X apuntando HACIA ARRIBA.",
        0
    );

    negative[0] = captureAccelAxis(
        "Coloca el sensor con -X apuntando HACIA ARRIBA.",
        0
    );

    positive[1] = captureAccelAxis(
        "Coloca el sensor con +Y apuntando HACIA ARRIBA.",
        1
    );

    negative[1] = captureAccelAxis(
        "Coloca el sensor con -Y apuntando HACIA ARRIBA.",
        1
    );

    positive[2] = captureAccelAxis(
        "Coloca el sensor con +Z apuntando HACIA ARRIBA.",
        2
    );

    negative[2] = captureAccelAxis(
        "Coloca el sensor con -Z apuntando HACIA ARRIBA.",
        2
    );

    for (int i = 0; i < 3; i++)
    {
        // Offset
        cal.accBiasG[i] =
            (positive[i] + negative[i]) * 0.5f;

        // Escala para que la separación sea exactamente 2 g
        float span =
            positive[i] - negative[i];

        if (fabsf(span) > 0.5f)
            cal.accScale[i] = 2.0f / span;
        else
            cal.accScale[i] = 1.0f;
    }

    Serial.println();
    Serial.println("Bias acelerometro [g]:");

    for (int i = 0; i < 3; i++)
        Serial.println(cal.accBiasG[i], 7);

    Serial.println("Scale acelerometro:");

    for (int i = 0; i < 3; i++)
        Serial.println(cal.accScale[i], 7);
}


// =====================================================
// Calibrar magnetómetro AK09916
// =====================================================

void calibrateMag()
{
    waitEnter(
        "CALIBRACION AK09916\n"
        "Durante 60 segundos rota lentamente el IMU en TODAS las "
        "orientaciones posibles.\n"
        "Haz movimientos tipo figura-8 y rota sobre X, Y y Z."
    );

    float minMag[3] =
    {
        100000.0f,
        100000.0f,
        100000.0f
    };

    float maxMag[3] =
    {
        -100000.0f,
        -100000.0f,
        -100000.0f
    };

    Serial.println("Comienza a mover el sensor...");

    uint32_t inicio = millis();

    while ((millis() - inicio) < 60000)
    {
        float a[3];
        float g[3];
        float m[3];

        if (!getSample(a, g, m))
            continue;

        // Ignorar lectura inválida
        if (m[0] == 0.0f &&
            m[1] == 0.0f &&
            m[2] == 0.0f)
            continue;

        for (int i = 0; i < 3; i++)
        {
            if (m[i] < minMag[i])
                minMag[i] = m[i];

            if (m[i] > maxMag[i])
                maxMag[i] = m[i];
        }
    }

    // Hard-iron
    float radius[3];

    for (int i = 0; i < 3; i++)
    {
        cal.magBiasUT[i] =
            (maxMag[i] + minMag[i]) * 0.5f;

        radius[i] =
            (maxMag[i] - minMag[i]) * 0.5f;
    }

    // Radio medio
    float avgRadius =
        (radius[0] + radius[1] + radius[2])
        / 3.0f;

    // Matriz identidad primero
    for (int i = 0; i < 9; i++)
        cal.magMatrix[i] = 0.0f;

    // Soft-iron diagonal
    cal.magMatrix[0] =
        avgRadius / radius[0];

    cal.magMatrix[4] =
        avgRadius / radius[1];

    cal.magMatrix[8] =
        avgRadius / radius[2];

    Serial.println();
    Serial.println("Hard-iron [uT]:");

    Serial.print("X = ");
    Serial.println(cal.magBiasUT[0], 6);

    Serial.print("Y = ");
    Serial.println(cal.magBiasUT[1], 6);

    Serial.print("Z = ");
    Serial.println(cal.magBiasUT[2], 6);

    Serial.println();
    Serial.println("Soft-iron diagonal:");

    Serial.println(cal.magMatrix[0], 6);
    Serial.println(cal.magMatrix[4], 6);
    Serial.println(cal.magMatrix[8], 6);
}


// =====================================================
// Aplicar calibración
// =====================================================

void applyCalibration(
    const float rawAcc[3],
    const float rawGyro[3],
    const float rawMag[3],
    float acc[3],
    float gyro[3],
    float mag[3])
{
    // Acelerómetro
    for (int i = 0; i < 3; i++)
    {
        acc[i] =
            (rawAcc[i] - cal.accBiasG[i])
            * cal.accScale[i];

        gyro[i] =
            rawGyro[i] - cal.gyroBiasDps[i];
    }

    // Hard iron
    float m[3];

    for (int i = 0; i < 3; i++)
        m[i] = rawMag[i] - cal.magBiasUT[i];

    // Matriz soft-iron
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
// Estimar ruido / covarianzas
// =====================================================

void estimateNoise()
{
    waitEnter(
        "CALCULO DE COVARIANZAS\n"
        "Deja nuevamente el sensor quieto durante la medicion."
    );

    RunningStats gyroStats[3];
    RunningStats accStats[3];

    const int N = 2000;

    Serial.println("Midiendo ruido...");

    for (int n = 0; n < N;)
    {
        float rawA[3];
        float rawG[3];
        float rawM[3];

        if (!getSample(rawA, rawG, rawM))
            continue;

        float a[3];
        float g[3];
        float m[3];

        applyCalibration(
            rawA, rawG, rawM,
            a, g, m
        );

        for (int i = 0; i < 3; i++)
        {
            float gyroRad =
                g[i] * DEG2RAD;

            float accMS2 =
                a[i] * G_MS2;

            gyroStats[i].add(gyroRad);
            accStats[i].add(accMS2);
        }

        n++;
        delay(2);
    }

    for (int i = 0; i < 3; i++)
    {
        cal.gyroVariance[i] =
            max(
                (float)gyroStats[i].variance(),
                1e-7f
            );

        cal.accVariance[i] =
            max(
                (float)accStats[i].variance(),
                1e-5f
            );
    }

    Serial.println("Varianza gyro [(rad/s)^2]:");

    for (int i = 0; i < 3; i++)
        Serial.println(cal.gyroVariance[i], 9);

    Serial.println("Varianza accel [(m/s^2)^2]:");

    for (int i = 0; i < 3; i++)
        Serial.println(cal.accVariance[i], 9);
}


// =====================================================
// Guardar NVS
// =====================================================

void saveCalibration()
{
    cal.magic = CAL_MAGIC;
    cal.version = CAL_VERSION;
    cal.reserved = 0;

    prefs.begin("imuCal", false);

    size_t escritos =
        prefs.putBytes(
            "blob",
            &cal,
            sizeof(cal)
        );

    prefs.end();

    Serial.println();

    if (escritos == sizeof(cal))
        Serial.println("CALIBRACION GUARDADA EN NVS.");
    else
        Serial.println("ERROR GUARDANDO CALIBRACION.");
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("==============================");
    Serial.println(" CALIBRACION ICM-20948");
    Serial.println("==============================");

    initIMU();

    calibrateGyro();
    calibrateAccel();
    calibrateMag();
    estimateNoise();

    saveCalibration();

    Serial.println();
    Serial.println("==============================");
    Serial.println(" CALIBRACION FINALIZADA");
    Serial.println("==============================");
    Serial.println();
    Serial.println(
        "Ahora carga el firmware micro-ROS."
    );
}

void loop()
{
}