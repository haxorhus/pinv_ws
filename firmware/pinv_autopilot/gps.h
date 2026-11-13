// =====================================================
// gps.h — GPS RTK u-blox ZED-F9P por UART1 (UBX-NAV-PVT)
//
// Librería: "SparkFun u-blox GNSS v3" (Library Manager).
// No se guarda nada en la flash del GPS (sin saveConfiguration):
// la configuración va a RAM/BBR y se repite en cada conexión.
// =====================================================
#pragma once

#include <Arduino.h>

// Una solución de navegación, tal como la entrega el receptor.
struct GpsSample
{
    double lat;         // [deg] WGS84
    double lon;         // [deg]
    double alt;         // [m] sobre el elipsoide WGS84 (lo que pide NavSatFix)
    float  hAcc;        // [m] precisión horizontal estimada (1 sigma)
    float  vAcc;        // [m] precisión vertical estimada
    uint8_t fixType;    // 0 sin fix, 2 = 2D, 3 = 3D, 4 = GNSS+DR, 5 = solo tiempo
    bool    fixOk;      // flag gnssFixOK de NAV-PVT
    uint8_t carrSoln;   // 0 sin RTK, 1 = RTK float, 2 = RTK fixed
    uint8_t siv;        // satélites usados
};

// Abre UART1. Llamar en setup(); la conexión con el receptor se hace
// después, desde gpsUpdate(), sin bloquear el arranque.
void gpsInit();

// Conecta/reconecta con el receptor y lee si llegó una solución nueva.
// Llamar en cada vuelta de loop(). true = 'out' tiene un PVT nuevo.
bool gpsUpdate(GpsSample &out);

// true si el receptor respondió y está mandando PVT.
bool gpsConnected();
