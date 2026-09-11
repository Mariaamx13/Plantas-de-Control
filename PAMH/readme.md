# Péndulo Amortiguado a Hélice: Identificación y Control (PAMH)

## Descripción del Proyecto

El Péndulo Amortiguado a Hélice (PAMH) es un sistema de control de posición angular que utiliza un motor sin escobillas (BLDC) con una hélice para generar empuje vertical y un sensor óptico incremental (USDigital E5, 3600 CPR) para medir el ángulo. El sistema presenta dinámicas subamortiguadas y comportamientos no lineales que desafían el control lineal clásico.

### Especificaciones del Sistema

- **Motor:** BLDC sin escobillas
- **Actuador:** Hélice de empuje variable
- **Sensor:** Codificador óptico incremental (3600 CPR)
- **Punto de operación:** 0.6 rad
- **Rango de medición:** ±π rad

## Metodología

### 1. Identificación del Modelo

Los datos experimentales fueron capturados en lazo abierto mediante excitación escalón alrededor del punto de operación 0.6 rad con perturbaciones.

**Procedimiento:**
- Importación en MATLAB con `systemIdentification`
- Evaluación de modelos de segundo orden (P2U, P2DU, P2DZU)
- Selección del modelo P2U (85.81% de ajuste, mejor relación complejidad-precisión)

**Modelo Identificado:**

$$\text{Angulo}(s) = \frac{5.3645}{s^2 + 0.2517s + 15.26}$$

**Parámetros clave:**
- Ganancia DC: 5.3645 rad/entrada
- Frecuencia natural: ωₙ = √15.26 ≈ 3.91 rad/s
- Factor de amortiguamiento: ζ = 0.2517 / (2 × 3.91) ≈ 0.032 (muy subamortiguado)

**Representación en Espacio de Estados:**

$$\mathbf{A} = \begin{bmatrix} 0 & 1 \\ -15.26 & -0.2517 \end{bmatrix}, \quad \mathbf{B} = \begin{bmatrix} 0 \\ 5.3645 \end{bmatrix}, \quad \mathbf{C} = [1 \; 0]$$

### 2. Especificaciones de Diseño

- Tiempo de estabilización: **ts < 5 s** (criterio 2%)
- Sobreimpulso máximo: **Mp < 3%**
- Error estado estacionario: **Ess = 0**

## Controladores Diseñados

### 3.1 REI por Ubicación de Polos

**Estrategia:** Asignación manual de polos del sistema aumentado (con integrador).

**Sistema Aumentado (orden 3):**

$$\mathbf{A}_s = \begin{bmatrix} 0 & 1 & 0 \\ -15.26 & -0.2517 & 0 \\ -1 & 0 & 0 \end{bmatrix}, \quad \mathbf{B}_s = \begin{bmatrix} 0 \\ 5.3645 \\ 0 \end{bmatrix}$$

**Polos Seleccionados:**
$$P_s = [-0.89 + 0.62i, \; -0.89 - 0.62i, \; -4.5]$$

Dos polos complejos dominantes (amortiguamiento moderado) + un polo real no dominante (5× más rápido).

**Ganancias Obtenidas** (con `acker()`):
$$K = [-1.1322 \; 1.1237], \quad K_i = 0.9869$$

**Desempeño en Simulación:**
- Tiempo de estabilización: **4.95 s** ✓
- Sobreimpulso: **≈ 0%** ✓
- Rechazo a perturbaciones: Efectivo ✓

**Desempeño Experimental:**
- Tiempo de estabilización: **5.26 s** (~6% de degradación)
- Sobreimpulso: **13.67%** (vs. 0% simulado)
- Causa de desviación: No linealidades (empuje variable con ángulo), ruido del sensor, corrientes de aire

---

### 3.2 Controlador I-PD por LQR

**Estrategia:** Optimización cuadrática con penalización explícita de esfuerzo de control.

**Índice de Costo:**
$$J = \int_0^\infty (x^T Q x + u^T R u) \, dt$$

**Matrices de Ponderación:**
$$Q = \text{diag}([10, 1, 10]), \quad R = 0.5$$

Pesos altos en posición (10) e integral del error (10); pesos bajos en velocidad (1) y control (0.5).

**Ganancias LQR Obtenidas:**

| Ganancia | Valor |
|----------|-------|
| Kp | 4.05 |
| Ki | -3.16 |
| Kd | 4.23 |

Interpretación: acción proporcional fuerte, integral negativa (corrección anticipada), derivativa moderada.

**Desempeño en Simulación:**
- Tiempo de estabilización: **< 5 s** ✓
- Sobreimpulso: **< 3%** ✓
- Respuesta suave, bien amortiguada

**Desempeño Experimental:**
- Transitorio pronunciado inicialmente (t = 0–2 s)
- Estabilización alrededor de 0.6 rad con pequeñas oscilaciones residuales
- Excelente rechazo a perturbaciones (desviación mínima cuando se aplica perturbación externa)
- Cumple especificaciones con margen: ts ≈ 3.88 s, Mp ≈ 8%

---

### 3.3 PID Continuo por IMC (Internal Model Control)

**Estrategia:** Método IMC ajustado iterativamente en `sisotool`.

**Compensador Inicial (IMC):**
$$C_0 = 0.079321 \cdot \frac{s^2 + 0.252s + 15.3}{s(s + 1.3)}$$

Incumplía especificaciones (ts > 5 s).

**Compensador Ajustado:**
$$C = 0.1793 \cdot \frac{s^2 + 0.252s + 15.3}{s(s + 1.77)}$$

Modificación de polo (1.3 → 1.77) para cumplir ts = 4.84 s.

**Desempeño en Simulación:**
- Tiempo de estabilización: **4.84 s** ✓
- Rechazo a perturbaciones: Efectivo durante ventana activa

**Desempeño Experimental:**
- **Falla crítica:** Sistema subamortiguado, oscilaciones sostenidas alrededor de 0.6 rad
- Amplitud de oscilación: constante, no amortigua
- Comportamiento: responde al error pero sin anticipación de dinámicas
- **Conclusión:** PID clásico ineficiente para PAMH

---

### 3.4 PID en Tiempo Discreto

**Tiempo de muestreo:** Ts = 0.02 s

**Modelo Discretizado** (con `c2d`):
$$G_d(z) = \frac{0.0010706(z + 0.9983)}{z^2 - 1.989z + 0.995}$$

**Diseño:** Cancelación de polos dominantes + integrador + polo real.

**Compensador Discreto:**
$$C_d(z) = \frac{0.21268(z^2 - 1.988z + 0.994)}{(z - 1)(z - 0.96)}$$

**Desempeño en Simulación:**
- Tiempo de estabilización: **4.98 s** ✓
- Comportamiento aproximado al continuo

**Desempeño Experimental:**
- **Comportamiento altamente oscilatorio**
- Causa: retardo de muestreo (0.02 s) introduce sobrecorrección
- Efectos: Menor amortiguamiento que PID continuo (discretización reduce margen de estabilidad)
- Control actualiza solo cada 0.02 s → retraso en acción correctiva
- **Conclusión:** Discretización reduce robustez para este sistema rápido

---

## Resultados Comparativos

### Simulación (MATLAB/Simulink)

| Característica | PID | I-PD | REI | PID Discreto |
|---|---|---|---|---|
| **Tiempo est. (s)** | 4.84 | 3.88 | 4.95 | 4.98 |
| **Sobreimpulso (%)** | 0.129 | 8 | 13 | 1 |
| **Oscilaciones** | Sí | Sí | No | Sí |
| **Estabilidad** | Marginal | Buena | Excelente | Buena |

### Experimento (Planta Física)

| Característica | PID | I-PD | REI | PID Discreto |
|---|---|---|---|---|
| **Estabilización** | ✗ Falla | ✓ Exitosa | ✓ Exitosa | ✗ Oscilante |
| **Oscilaciones** | Sostenidas | Residuales | Mínimas | Amplias |
| **Rechazo pert.** | Pobre | Excelente | Excelente | Pobre |
| **Comportamiento** | Subamortiguado | Suave, rápido | Estable | Oscilatorio |

### Ranking de Desempeño Experimental

1. **REI (Mejor):** Respuesta más estable, mejor amortiguamiento, rechazo robusto a perturbaciones
2. **I-PD:** Rápido, buen rechazo de perturbaciones, pero con oscilaciones residuales iniciales
3. **PID Continuo:** Oscilaciones sostenidas, amortiguamiento insuficiente
4. **PID Discreto:** Altamente oscilatorio, retardo acumula errores

---

## Discrepancias Simulación–Experimento

**Causas Identificadas:**

1. **No linealidades:** Empuje de hélice varía con ángulo de operación (no capturado en P2U)
2. **Ruido del sensor:** Codificador óptico introduce perturbaciones de cuantización
3. **Condiciones ambientales:** Corrientes de aire, fricción, zona muerta del motor
4. **Dinámicas no modeladas:** Inercia variable, respuesta transitoria del BLDC
5. **Efectos discretos:** Retardo de muestreo en tiempo discreto amplía error

**Impacto:**
- REI: Tolerante a no linealidades por realimentación de estado completa
- PID/I-PD: Sensibles al modelo; I-PD mejor que PID puro por término derivativo anticipativo
- PID Discreto: Vulnerable a efectos de muestreo en sistemas rápidos (ωₙ ≈ 3.91 rad/s)

