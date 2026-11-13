// =====================================================
// gps.cpp — GPS RTK u-blox ZED-F9P por UART1 (UBX-NAV-PVT)
// =====================================================
#include "gps.h"

#include <SparkFun_u-blox_GNSS_v3.h>

#include "config.h"

static HardwareSerial GPSSerial(1);
static SFE_UBLOX_GNSS_SERIAL gnss;

static bool connected = false;
static uint32_t lastAttemptMs = 0;
static uint32_t lastPvtMs = 0;

void gpsInit()
{
    GPSSerial.setRxBufferSize(1024);    // antes de begin()
    GPSSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
}

bool gpsConnected()
{
    return connected;
}

// Configuración en RAM/BBR (capa por defecto de la librería), sin
// tocar la flash del receptor.
static bool configureReceiver()
{
    if (!gnss.begin(GPSSerial, GPS_BEGIN_WAIT_MS))
        return false;

    bool ok = gnss.setUART1Output(COM_TYPE_UBX);    // UART1: solo UBX, sin NMEA
    ok &= gnss.setNavigationFrequency(GPS_NAV_HZ);
    ok &= gnss.setAutoPVT(true);                    // el receptor manda PVT solo
    return ok;
}

bool gpsUpdate(GpsSample &out)
{
    const uint32_t now = millis();

    if (!connected)
    {
        if ((uint32_t)(now - lastAttemptMs) < GPS_RETRY_MS)
            return false;
        lastAttemptMs = now;

        // gnss.begin() bloquea loop() hasta GPS_BEGIN_WAIT_MS. Solo se
        // intenta si por la UART llega algo: con el GPS desconectado no
        // se frena la IMU ni micro-ROS cada segundo.
        if (GPSSerial.available() == 0)
            return false;

        connected = configureReceiver();
        lastPvtMs = now;
        return false;
    }

    // Con autoPVT, getPVT() no bloquea: procesa lo que haya en la UART
    // y devuelve true solo si se completó un NAV-PVT nuevo.
    if (!gnss.getPVT())
    {
        if ((uint32_t)(now - lastPvtMs) > GPS_TIMEOUT_MS)
            connected = false;      // cable suelto o receptor reiniciado
        return false;
    }

    lastPvtMs = now;

    out.lat      = gnss.getLatitude() * 1e-7;
    out.lon      = gnss.getLongitude() * 1e-7;
    out.alt      = gnss.getAltitude() / 1000.0;             // mm -> m (elipsoide)
    out.hAcc     = gnss.getHorizontalAccEst() / 1000.0f;    // mm -> m
    out.vAcc     = gnss.getVerticalAccEst() / 1000.0f;
    out.fixType  = gnss.getFixType();
    out.fixOk    = gnss.getGnssFixOk();
    out.carrSoln = gnss.getCarrierSolutionType();
    out.siv      = gnss.getSIV();

    return true;
}
