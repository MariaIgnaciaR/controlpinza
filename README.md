# Prototipo: Sistema de Transferencia Automatizada de Probetas (3 GDL + Pinza)

Versión prototipo de un sistema robótico autónomo para el **traslado automatizado de probetas de laboratorio** (identificadas por códigos de color rojo y azul) desde un rack de origen hasta un rack de destino. El sistema integra visión artificial por computador (OpenCV), cinemática directa e inversa, y control coordinado de motores Dynamixel y un servomotor de pinza.

---

## Descripción del Prototipo y Objetivos
*   **Detección Automática:** La cámara identifica de forma autónoma la posición de las probetas sobre la zona de trabajo (marcaras en rojo y azul).
*   **Selección y Traslado:** El brazo calcula la cinemática inversa, se aproxima de manera aérea, desciende, acciona la pinza para sujetar la probeta, la levanta y está listo para transferirla hacia el rack de destino.
*   **Entorno de Trabajo:** Diseñado para operar con bandejas o racks calibrados mediante un sistema de referencia espacial absoluto.

---

## Arquitectura de Hardware y Conexiones

El hardware se encuentra desacoplado mediante fuentes de alimentación independientes para garantizar la estabilidad eléctrica y proteger los componentes de control:

*   **Motores Dynamixel (Base, Hombro, Codo):** 
    *   Alimentados mediante una **fuente externa de 10V**.
    *   Controlados a través de un **U2D2 USB Converter** y un **U2D2 Power Hub**.
    *   *Nota:* La articulación de la base opera en un rango de rotación de 0° a 180°.
*   **Pinza (Servomotor MG996R):**
    *   Alimentado por una **fuente externa independiente de 5.5V**.
    *   Conectado al **Pin 9** de la placa Arduino para el control de apertura y cierre mediante PWM.
    *   **Gestión de Masas:** Se utiliza **GND común** exclusivamente entre la fuente de 5.5V y el Arduino/pinza. La fuente de 10V de los Dynamixel se mantiene aislada para evitar interferencias por ruido eléctrico.
*   **Sistema de Visión:** Cámara web USB superior orientada cenitalmente hacia la zona de los racks para el procesamiento de imagen en tiempo real.

---

## Modelado Matemático y Cinemática

El control de movimiento se fundamenta en la robótica de manipuladores:

1.  **Cinemática Inversa (IK):** 
    *   Traduce las coordenadas espaciales $(X, Y, Z)$ detectadas por la cámara a los ángulos articulares requeridos.
    *   Utiliza trigonometría para el posicionamiento de la base y la **Ley de Cosenos** para resolver la geometría del hombro y el codo, mapeándolos a *ticks* de los Dynamixel dentro de rangos seguros de operación.
2.  **Cinemática Directa (FK):** 
    *   Permite calcular la posición espacial $(X, Y, Z)$ a partir de la lectura de los ángulos actuales de los motores. Se emplea como rutina de validación de errores en el posicionamiento de la pinza.

---

## Estructura del Software

*   `tests.py` / `tests_2.py`: Script central en Python. Contiene el bucle de procesamiento de visión (OpenCV con máscaras HSV), la máquina de estados del ciclo de agarre/traslado, y la comunicación serial con el U2D2 y el Arduino.
*   `main.cpp`: Firmware de Arduino para la gestión del comando serial (`G<angulo>\n`) encargado de mover el servomotor MG996R de la pinza.

---

## ⌨️ Controles e Interacción (Modo Pruebas / Operador)

Durante la ejecución del script en Python, el sistema responde a los siguientes comandos por teclado:

*   **`[ESPACIO]`** : Inicia el ciclo automático de detección, aproximación aérea, descenso, agarre de la probeta e izado.
*   **`[O]`** : Abre la pinza (liberar probeta en rack destino).
*   **`[G]`** : Cierra la pinza (sujetar probeta).
*   **`[T]`** : Alterna el filtro de selección de objetivo (`AMBOS` / `ROJO` / `AZUL`).
*   **`[C]`** : Selecciona la cámara activa.
*   **`[M]`** : Muestra u oculta las máscaras de segmentación de color en pantalla.
*   **`[+]` / `[-]`** : Ajusta en tiempo real (±5 mm) la altura de agarre (`Z_GRIP`).
*   **`[H]`** : Retorna el brazo a la postura segura de origen (`HOME`).
*   **`[Q]`** : Finaliza y cierra la ejecución del programa.
