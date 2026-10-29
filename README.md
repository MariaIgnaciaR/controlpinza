# Prototipo: Sistema de Transferencia Automatizada de Probetas (3 GDL + Pinza)

**Estado del proyecto: en desarrollo.**

Prototipo de un sistema robótico para el traslado automatizado de probetas de laboratorio, identificadas por color (rojo y azul), desde un rack de origen hasta un rack de destino. Integra visión artificial (OpenCV), cinemática directa e inversa, y control coordinado de tres motores Dynamixel y un servomotor de pinza.

## Objetivo

- **Detección automática:** la cámara identifica por color la posición de las probetas en la zona de trabajo. Las posiciones no se graban: se calculan en cada ejecución a partir de la imagen.
- **Selección y traslado:** el operador elige el color (rojo o azul). El brazo calcula la cinemática inversa, se aproxima por aire, desciende, cierra la pinza, levanta la probeta y la traslada al rack de destino.
- **Entorno de trabajo:** bandejas o racks ubicados dentro del alcance del brazo, con un sistema de referencia espacial común entre la cámara y la base del robot.

## Estado actual

**Implementado**

- Cinemática directa e inversa del brazo (`cinematica.py`), con conversión entre ángulos y ticks basada en calibración medida en el robot real.
- Calibración guiada de las tres articulaciones (`calibrar.py`) y verificación de puntos con medición en regla (`probar_punto.py`).
- Detección de probetas rojas y azules por segmentación HSV, conversión de píxeles a milímetros y ciclo de aproximación, descenso, agarre y levantado (`robot_principal.py`).
- Simulación del brazo en pygame (`simulacion_brazo.py`), incluido el ciclo completo de traslado de una probeta (rojo o azul) hacia el rack de destino.
- Firmware de Arduino para la pinza (`platformio/src/main.cpp`).

**En desarrollo**

- Traslado completo en el robot real: actualmente el ciclo del script del robot termina con la probeta levantada, y la liberación en el rack de destino se hace manualmente con la tecla `O`.
- Detección del rack de destino y de sus huecos libres mediante visión, de modo que la posición de depósito tampoco sea fija. En la simulación, los racks se representan con coordenadas de referencia definidas en el código.
- Espera de llegada de cada movimiento leyendo la posición real de los motores, en lugar de tiempos fijos.
- Ajuste fino de alturas de agarre y de depósito (`Z_GRIP`) sobre el hardware definitivo.

## Arquitectura de hardware

El hardware está separado en fuentes de alimentación independientes para mantener la estabilidad eléctrica y proteger los componentes de control.

**Motores Dynamixel (base, hombro y codo)**

- Alimentados por una fuente externa de 10 V.
- Controlados mediante un U2D2 (convertidor USB) y un U2D2 Power Hub.
- La articulación de la base tiene un rango de giro limitado (no alcanza 180° completos); sus límites reales quedan registrados en la calibración.

**Pinza (servomotor MG996R)**

- Alimentada por una fuente externa independiente de 5.5 V.
- Señal conectada al pin 9 del Arduino (PWM) para apertura y cierre.
- Masas: GND común únicamente entre la fuente de 5.5 V, la pinza y el Arduino. La fuente de 10 V de los Dynamixel se mantiene aislada de ese GND para evitar ruido eléctrico.

**Visión**

- Cámara web USB superior, orientada cenitalmente hacia la zona de los racks.

## Modelado matemático

El brazo se modela en dos partes: la base gira en el plano horizontal y apunta hacia el objetivo; el hombro y el codo forman un brazo plano de dos eslabones (L1 y L2) en el plano vertical.

**Conversión ángulo a tick (calibrada en el robot)**

```
tick = cero + signo * ticks_por_grado * angulo
```

Los valores de `cero`, `signo`, `ticks_por_grado` y los límites seguros de cada motor se miden con `calibrar.py` y se guardan en `calibracion.json`.

**Cinemática inversa (IK):** de (x, y, z) a ticks

1. Base: `theta_base = atan2(y, x)`.
2. Reducción al plano vertical: `R = sqrt(x^2 + y^2)`, `dz = z - Z_HOMBRO`, `D = sqrt(R^2 + dz^2)`. Hay solución solo si `|L1 - L2| <= D <= L1 + L2`.
3. Codo (ley de cosenos): `cos(theta_codo) = (D^2 - L1^2 - L2^2) / (2 * L1 * L2)`, donde 0 grados es el brazo estirado.
4. Hombro: `theta_hombro = alpha + beta`, con `alpha = atan2(dz, R)` y `cos(beta) = (D^2 + L1^2 - L2^2) / (2 * L1 * D)`. Corresponde a la configuración "codo arriba".
5. Conversión de ángulos a ticks con la calibración y recorte a los límites seguros.

**Cinemática directa (FK):** de ticks a (x, y, z)

```
R = L1 * cos(theta_h) + L2 * cos(theta_h - theta_c)
z = Z_HOMBRO + L1 * sin(theta_h) + L2 * sin(theta_h - theta_c)
x = R * cos(theta_b)        y = R * sin(theta_b)
```

Se usa para verificar la IK: si la posición calculada difiere de la pedida, algún límite recortó el movimiento. También alimenta la simulación.

## Estructura del proyecto

| Archivo | Función |
|---|---|
| `cinematica.py` | Cinemática directa e inversa, conversión ticks/ángulos, límites y postura HOME. Lee `calibracion.json`. |
| `calibrar.py` | Calibración guiada: mide ceros, sentidos, ticks por grado, límites seguros y HOME. Genera `calibracion.json`. |
| `probar_punto.py` | Envía el brazo a un punto (x, y, z) y compara con la predicción de la cinemática directa. |
| `robot_principal.py` | Programa principal: visión (OpenCV, HSV), máquina de estados de agarre y comunicación con el U2D2 y el Arduino. |
| `simulacion_brazo.py` | Simulación en pygame del brazo, con ciclo de traslado entre racks. |
| `platformio/src/main.cpp` | Firmware de Arduino: recibe `G<angulo>\n` por serial y mueve el servo de la pinza. |

Si los nombres de archivo en el repositorio son distintos, ajustar esta tabla.

## Puesta en marcha

1. Instalar dependencias:
   ```
   pip install opencv-python numpy pyserial dynamixel-sdk pygame
   ```
2. Medir con regla, de eje a eje, `L1`, `L2` y `Z_HOMBRO`, y escribirlos en `cinematica.py`.
3. Compilar y cargar el firmware del Arduino con PlatformIO (ajustar `board` en `platformio.ini`).
4. Con los Dynamixel energizados y el brazo sujeto, ejecutar `python calibrar.py` y seguir las instrucciones.
5. Verificar con `python probar_punto.py` (comenzar con z alta y bajar de a poco).
6. Ajustar en `robot_principal.py`: `MM_PER_PIXEL`, `DIST_BASE_CAMARA`, `INVERTIR_Y`, `PINZA_ABIERTA`, `PINZA_CERRADA` y `Z_GRIP`.
7. Ejecutar `python robot_principal.py`.

Sin `calibracion.json`, `robot_principal.py` no mueve los motores.

## Controles

**Robot real (`robot_principal.py`)**

| Tecla | Acción |
|---|---|
| `ESPACIO` | Inicia el ciclo: detección, aproximación aérea, descenso, agarre y levantado. |
| `O` | Abre la pinza (liberar la probeta). |
| `G` | Cierra la pinza. |
| `T` | Alterna el filtro de objetivo: AMBOS, ROJO o AZUL. |
| `C` | Cambia la cámara activa. |
| `M` | Muestra u oculta las máscaras de color. |
| `+` / `-` | Sube o baja la altura de agarre (`Z_GRIP`) en 5 mm. |
| `H` | Vuelve a la postura HOME. |
| `Q` | Cierra el programa. |

**Simulación (`simulacion_brazo.py`)**

| Entrada | Acción |
|---|---|
| `1` o `R` | Traslada la probeta roja al rack de destino. |
| `2` o `A` | Traslada la probeta azul al rack de destino. |
| `ESPACIO` | Secuencia de agarre en la posición actual. |
| Arrastrar con el mouse en las vistas | Mueve el objetivo (cinemática inversa). |
| Rueda del mouse | Sube o baja Z. |
| Flechas y `Z` / `X` | Mueven base, hombro y codo por ticks (cinemática directa). |
| `O` / `G` | Abre o cierra la pinza. |
| `H` | Vuelve a HOME. |
| `ESC` | Sale. |

## Limitaciones conocidas

- El brazo tiene tres grados de libertad y la pinza va fija al segundo eslabón, por lo que la orientación de la pinza no es controlable de forma independiente.
- La precisión depende de la calibración y de las medidas de `L1`, `L2` y `Z_HOMBRO`.
- La conversión de píxeles a milímetros supone una cámara cenital con el eje de la imagen alineado con el eje X del robot.
- Prototipo en desarrollo: debe usarse con el brazo vigilado y con acceso rápido al corte de alimentación.
