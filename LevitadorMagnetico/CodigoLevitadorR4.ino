#include <Arduino.h>
#include "pwm.h"

// ── Pines ────────────────────────────────────────────────────────
#define PIN_HALL   A0

// ── Parámetros de muestreo ───────────────────────────────────────
#define T  0.002f                  // período [s] — 2 ms

// ── Punto de operación ───────────────────────────────────────────
#define CONSIGNA          800.0f   // posición de levitación [cuentas ADC]
#define OFFSET_EQUILIBRIO  49.0f   // duty cycle en equilibrio [%DC]

// ── Ganancias PID ────────────────────────────────────────────────
float Kp =  25.0f;
float Ki = 8.0f;
float Kd =    10.0f;

// ── Anti-windup ──────────────────────────────────────────────────
#define INTEGRAL_MAX  ( 100.0f / Ki)
#define INTEGRAL_MIN  (-100.0f / Ki)

// ── Variables de estado ──────────────────────────────────────────
float integral       = OFFSET_EQUILIBRIO / Ki;
float error_anterior = 0.0f;

// ── PWM (R4: pwm.h, 1kHz, pin D9) ───────────────────────────────
PwmOut pwm(D9);

// ================================================================
//  setup
// ================================================================
void setup()
{
    Serial.begin(115200);

    // R4: PwmOut con frecuencia 1000 Hz, duty inicial = offset
    pwm.begin(1000.0f, OFFSET_EQUILIBRIO);
    delay(500);

    Serial.println("t_ms,lectura,consigna,error,duty_cycle");
}

// ================================================================
//  loop — ejecuta exactamente cada 2 ms usando micros()
// ================================================================
void loop()
{
    static unsigned long t_anterior_us = 0;
    unsigned long ahora_us = micros();

    if (ahora_us - t_anterior_us < 2000UL) return;
    t_anterior_us = ahora_us;

    // ── 1. Leer sensor Hall ──────────────────────────────────────
    float lectura = (float)analogRead(PIN_HALL);

    // ── 2. Error ─────────────────────────────────────────────────
    float error = lectura - CONSIGNA;

    // ── 3. Integral con anti-windup ──────────────────────────────
    integral += error * T;
    if (integral >  INTEGRAL_MAX) integral =  INTEGRAL_MAX;
    if (integral <  INTEGRAL_MIN) integral =  INTEGRAL_MIN;

    // ── 4. Derivada ──────────────────────────────────────────────
    float derivada = (error - error_anterior) / T;

    // ── 5. Salida PID ────────────────────────────────────────────
    float duty_cycle = OFFSET_EQUILIBRIO
                     + Kp * error
                     + Ki * integral
                     + Kd * derivada;

    // ── 6. Saturación ────────────────────────────────────────────
    if (duty_cycle > 100.0f) duty_cycle = 100.0f;
    if (duty_cycle <   0.0f) duty_cycle =   0.0f;

    // ── 7. Aplicar PWM ───────────────────────────────────────────
    pwm.pulse_perc(duty_cycle);

    // ── 8. Guardar estado ────────────────────────────────────────
    error_anterior = error;

}