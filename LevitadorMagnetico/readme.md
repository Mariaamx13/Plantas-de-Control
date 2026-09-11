# Levitador Magnético Controlado con PID

Diseño, simulación e implementación de un sistema de levitación magnética utilizando un controlador PID con Arduino UNO R4 WiFi.

## Descripción del Proyecto

Un levitador magnético es un sistema inherentemente inestable que utiliza fuerzas electromagnéticas para suspender un objeto en el aire sin contacto mecánico. Este proyecto desarrolla un controlador por realimentación para estabilizar un imán permanente a aproximadamente 3 cm de distancia bajo una bobina electromagnética.

### Componentes del Sistema

- **Actuador:** Bobina Bourns 1140-153K-RC (choke de RF con núcleo de ferrita)
- **Sensor:** Hall effect SS49E (medición de posición por intensidad de campo magnético)
- **Plataforma de Control:** Arduino UNO R4 WiFi (microcontrolador RA4M1, 48 MHz, FPU)
- **Masa del imán:** 2.4 g

## Metodología

### 1. Modelado Empírico

Se realizaron mediciones de fuerza magnética en función de corriente y distancia (19 puntos experimentales en el rango 2.7–5.6 cm y 0–40% duty cycle). Usando regresión lineal múltiple sobre el modelo:

$$F_{mag}(i, x) = \frac{C \cdot i^\alpha}{x^\beta}$$

**Parámetros ajustados:**
- C = 4.082 × 10⁻⁷
- α = 0.354
- β = 3.323
- R² = 0.87

### 2. Linealización y Función de Transferencia

Se linealizó alrededor del punto de operación x₀ = 3 cm, i₀ = 143 mA:

$$G(s) = \frac{\Delta X(s)}{\Delta I(s)} = \frac{13.63}{s^2 - 1088.75}$$

Polos en s = ±33.00 rad/s, confirmando inestabilidad inherente.

### 3. Diseño del Controlador PID

Usando ubicación de polos con especificaciones:
- Sobreimpulso máximo: 10%
- Tiempo de establecimiento (criterio 2%): 0.1 s
- Factor de amortiguamiento: ζ = 0.5912
- Frecuencia natural: ωₙ = 67.66 rad/s

**Ganancias obtenidas:**
- **Kp** = 1589.67
- **Ki** = 67181.83
- **Kd** = 20.54

### 4. Implementación en Arduino

- **Período de muestreo:** Ts = 2 ms (criterio Ts ≤ τ = 30.3 ms)
- **Frecuencia PWM:** 1 kHz
- **Integrador:** Euler hacia adelante con anti-windup (límite ±100/Ki)
- **Derivador:** Diferencia hacia atrás sin filtrado
- **Referencia:** 770 cuentas ADC (posición deseada ~3 cm)

### Simulación (Simulink)

✓ **Éxito:** El controlador estabilizó el modelo linealizado
- Tiempo de establecimiento: 0.078 s (dentro de especificación)
- Sobreimpulso: 23% (superior al diseño, debido a ceros del PID)
- Rechazo de perturbaciones: Correcto

## Recomendaciones para Trabajo Futuro

1. Verificar la interpretación de la polaridad de la realimentación del sensor Hall
2. Reducir el entrehierro de operación a distancias más pequeñas (< 2 cm) donde la influencia de la corriente sea más significativa
3. Caracterizar experimentalmente la dinámica eléctrica de la bobina (L, R) vs. frecuencia
4. Implementar un filtro paso-bajo (N ≥ 1000) en el término derivativo del Arduino
5. Considerar control no lineal o adaptativo que tolere variaciones paramétricas
6. Realizar ajustes empíricos iterativos de Kp, Ki, Kd si se resuelven las causas estructurales
