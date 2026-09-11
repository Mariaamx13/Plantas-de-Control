# Helicóptero Aterrizdo: Identificación y Control MIMO de Dos Grados de Libertad

Identificación experimental y diseño de reguladores para un helicóptero aterrizdo con acoplamiento entre ejes pitch y yaw.

**Autores:** María Fernanda Alvarado Moraga, Jose Pablo Nuñez García, Ana Victoria Rojas Lazo  
**Profesor:** Eduardo Interiano Salguero  
**Institución:** Instituto Tecnológico de Costa Rica (ITCR), Laboratorio de Control Automático (EL-5409)  
**Fecha:** Junio 2026

## Descripción del Proyecto

El Heli2DoF es una planta MIMO (Multi-Input Multi-Output) que consiste en un helicóptero aterrizdo con dos grados de libertad mecánicamente acoplados: **Pitch** (eje longitudinal) y **Yaw** (eje vertical), controlados independientemente mediante motores de corriente directa. Este sistema representa los desafíos prácticos del control multivariable en presencia de acoplamiento cruzado dinámico.

### Caracterización Experimental

- **Eje Pitch:** Dinámico de segundo orden subamortiguado
- **Eje Yaw:** Comportamiento integrador (sistema tipo-1)
- **Acoplamiento:** Motor de pitch influencia al eje yaw mediante un integrador puro
- **Sensores:** Codificadores ópticos incrementales (resolución 2π/3600 rad)
- **Actuadores:** Motores DC ±12 V con zona muerta y fricción seca

## Metodología

### 1. Identificación Experimental en Lazo Abierto

Se obtuvieron tres funciones de transferencia SISO mediante excitación de entrada y captura de respuesta angular:

**Eje Pitch / Entrada Pitch:**
$$G_{pitch/e\_pitch}(s) = \frac{0.11891(s + 0.3938)}{s^2 + 0.4745s + 1.493}$$

Identificada con la herramienta `ident` de MATLAB sobre datos truncados a 11 segundos.

**Eje Yaw / Entrada Yaw:**
$$G_{yaw/e\_yaw} = -\frac{0.06}{s(s + 0.2)}$$

Integrador más polo real (determinado con `sisotool`). El signo negativo refleja la convención de control.

**Acoplamiento: Pitch → Yaw:**
$$G_{yaw/e\_pitch} = \frac{0.0168}{s}$$

Integrador puro identificado mediante iteración manual del coeficiente kyp sobre los primeros 4 segundos.

**Acoplamiento inverso:** Despreciado por negligible en condiciones normales de operación.

### 2. Modelo MIMO en Variables de Estado

Cada función SISO fue convertida a forma canónica observable (OCF) usando `compreal()` de MATLAB, expandida a cuatro estados y integrada en una representación de sexto orden:

$$\mathbf{A} = \begin{bmatrix} 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \\ -1.4934 & 0 & -0.4745 & 0 \\ 0 & 0 & 0 & -0.2 \end{bmatrix}, \quad \mathbf{B} = \begin{bmatrix} 0.1189 & 0 \\ 0.0168 & 0 \\ -0.0096 & 0 \\ 0 & -0.06 \end{bmatrix}$$

$$\mathbf{C} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}, \quad \mathbf{D} = \mathbf{0}$$

### 3. Diseño de Reguladores REI (Realimentación de Estado Integral)

Se extendió el modelo a orden 6 agregando dos integradores del error para garantizar error de estado estacionario nulo.

#### Método 1: Ubicación de Polos

Se fijaron seis polos en lazo cerrado:
- **Polos dominantes:** $p_{1,2} = -0.5140 \pm 0.3000j$ (tiempo de asentamiento 8 s, amortiguamiento moderado)
- **Polos no dominantes:** $p_{3,4} = -2.5000, -2.5005$ y $p_{5,6} = -2.6000, -2.4000$ (cinco veces más rápidos)

**Ganancias obtenidas** con `place()`:

$$\mathbf{K} = \begin{bmatrix} 43.2820 & 9.3780 & 23.8622 & 1.8470 \\ 30.3019 & -140.4971 & 73.0365 & -87.9785 \end{bmatrix}$$

$$\mathbf{K}_i = \begin{bmatrix} -81.3668 & -11.9018 \\ -133.1413 & 40.9595 \end{bmatrix}$$

#### Método 2: Control Óptimo Cuadrático (LQR)

Minimización del índice de costo:
$$J = \int (x_a^T Q x_a + u^T R u) \, dt$$

**Matriz de pesos Q** (diagonal):
$$\mathbf{Q} = \text{diag}([1000, 75, 75, 100, 2500, 100])$$

Prioridad en pitch (pesos 1000 y 2500) y amortiguamiento suave de yaw.

**Matriz de esfuerzo:** $\mathbf{R} = \text{diag}([1, 1])$ (penalización moderada de control)

**Ganancias obtenidas** con `lqr()`:

$$\mathbf{K} = \begin{bmatrix} 40.642 & 1.5829 & 6.4999 & 0.9052 \\ 3.5579 & -26.4972 & 3.3589 & -28.187 \end{bmatrix}$$

$$\mathbf{K}_i = \begin{bmatrix} 49.8125 & 0.8652 \\ 4.3260 & -9.9625 \end{bmatrix}$$

## Resultados

### Simulación (Simulink)

Ambos controladores cumplieron las especificaciones en lazo lineal:
- Tiempo de estabilización < 8 s ✓
- Sobreimpulso < 5% ✓
- Error estado estacionario = 0 ✓
- Excursión yaw limitada ✓

### Validación Experimental

#### REI por Ubicación de Polos

| Métrica | Especificación | Resultado | Estado |
|---------|---|---|---|
| Sobreimpulso pitch | < 5% | 34% | ✗ Falla |
| Excursión yaw | < 0.1 rad | 0.21 rad | ✗ Falla |
| Error estado estacionario | 0 ± 0.003 rad | 0 | ✓ OK |
| Tiempo establecimiento | < 8 s | ~5 s | ✓ OK |

**Observaciones:**
- Respuesta rápida pero agresiva (ganancias elevadas)
- Rizado residual ±0.02 rad en pitch
- Acoplamiento intensificado: motor pitch induce oscilación en yaw de ±0.21 rad

#### REI por LQR

| Métrica | Especificación | Resultado | Estado |
|---------|---|---|---|
| Sobreimpulso pitch | < 5% | 17% | ✗ Falla |
| Excursión yaw | < 0.1 rad | 0.13 rad | ✗ Marginal |
| Error estado estacionario | 0 ± 0.003 rad | 0 | ✓ OK |
| Tiempo establecimiento | < 8 s | ~6 s | ✓ OK |

**Observaciones:**
- Transitorio más suave gracias a penalización de control
- Rizado residual ±0.04 rad (ciclo límite de ~1 Hz)
- Mejor desempeño acoplado: excursión yaw reducida a 0.13 rad (vs. 0.21 con ubicación)

### Análisis del Ciclo Límite

Ambos diseños exhiben un ciclo límite no previsto en simulación, atribuible a:
1. **Zona muerta** y **fricción seca** de los motores DC
2. **Acción integral** que continúa acumulando error hasta vencer la zona muerta
3. **Respuesta abrupta** del motor cuando se supera el umbral
4. Ciclo se repite a ~1 Hz con amplitud ±0.02–0.04 rad

Este fenómeno explica por qué el error medio cumple 0.003 rad pero el instantáneo lo excede periódicamente.

## Comparación: Ubicación de Polos vs. LQR

| Aspecto | Ubicación de Polos | LQR |
|---------|---|---|
| Velocidad | Más rápido (0.8 s cruce) | Más lento (1.5–2 s cruce) |
| Suavidad | Agresiva, ganancias altas | Moderada, equilibrio desempeño-esfuerzo |
| Sobreimpulso | 34% | 17% |
| Excursión yaw | 0.21 rad | 0.13 rad |
| Sintonización | Manual (6 polos) | Sistemática (Q, R) |
| Reproducibilidad | Media | Alta (parámetros físicos significativos) |

**Conclusión:** El LQR ofrece mejor compromiso y se aproxima más a las especificaciones críticas de acoplamiento.

## Discrepancias Simulación–Experimento

No modelados en identificación:
- **Zona muerta** y **fricción seca** de motores
- **Cuantización** del codificador óptico (2π/3600 rad ≈ 0.00175 rad)
- **Ruido** amplificado en derivadores con filtro de síntesis de velocidad
- **Acoplamiento inverso** (yaw → pitch) despreciado pero presente
- **Dinámicas** de alta frecuencia no capturadas en tiempo de experimento limitado

Resultado: Sobreimpulso y excursión yaw mayores a lo predicho; ciclo límite ausente en modelo lineal.
