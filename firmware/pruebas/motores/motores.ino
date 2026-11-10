
//------------------------------------------------------------------------------------------------------------

const int escPin = 4;
const int LED = 7;

const uint32_t PWM_FREQ = 50;
const uint8_t PWM_RES = 14;

const uint16_t ESC_MIN = 1500;
const uint16_t ESC_MAX = 1700;

// Máximo para 14 bits
const uint32_t PWM_MAX = (1UL << PWM_RES) - 1;


//------------------------------------------------
// Conversión de microsegundos a duty
//------------------------------------------------
uint32_t usToDuty(uint16_t us)
{
    return ((uint64_t)us * PWM_MAX) / 20000ULL;
}


//------------------------------------------------
// Enviar pulso al ESC
//------------------------------------------------
void setESC(uint16_t us)
{
    us = constrain(us, ESC_MIN, ESC_MAX);

    uint32_t duty = usToDuty(us);

    ledcWrite(escPin, duty);
}


void setup()
{
    // Utilizar XTAL de 40 MHz
    bool clockOK = ledcSetClockSource(LEDC_USE_XTAL_CLK);

    // Configurar LEDC
    bool pwmOK = ledcAttach(
        escPin,
        PWM_FREQ,
        PWM_RES
    );

    // Si falla la configuración
    if (!clockOK || !pwmOK)
    {
        pinMode(LED, OUTPUT);

        while (1)
        {
            digitalWrite(
                LED,
                !digitalRead(LED)
            );

            delay(100);
        }
    }

    // ESC detenido
    setESC(1500);

    // Esperar inicialización
    delay(5000);
}


void loop()
{
    // 1500 → 1700 µs
    for (uint16_t us = 1500; us <= 1700; us += 10)
    {
        setESC(us);
        delay(1000);
    }

    // Mantener 1700 µs
    delay(3000);

    // 1700 → 1500 µs
    for (uint16_t us = 1700; us >= 1500; us -= 10)
    {
        setESC(us);
        delay(1000);
    }

    // Mantener detenido
    delay(3000);
}

