import cv2
import numpy as np
import math
import time
import serial
from dynamixel_sdk import *

# La cinemática (IK, FK, límites, HOME y medidas L1, L2, Z_HOMBRO) vive en cinematica.py
from cinematica import calcular_ik, calcular_fk, HOME, CALIBRADO

# ==============================================================================
# 1. PARÁMETROS DE TRABAJO (mm)
# ==============================================================================
# Las medidas del brazo (L1, L2, Z_HOMBRO) y los límites de los motores se
# editan en cinematica.py / calibracion.json (generado por calibrar.py).

# Alturas de trabajo
Z_APPROACH = 105.0         # Altura aérea
Z_GRIP = 70.0              # Altura de agarre (sube si choca con la mesa, baja si no llega)
MM_PER_PIXEL = 0.55        # Calibra con una regla en la mesa: 100 mm / píxeles medidos
DIST_BASE_CAMARA = 150.0   # Distancia (mm) del eje de la base al punto de la mesa en el centro de la imagen

INVERTIR_Y = True          # True: corrige el giro hacia el lado contrario

# ==============================================================================
# 1b. CÁMARA Y COLORES
# ==============================================================================
CAMARA_INDEX = 0

RANGOS_COLOR = {
    "ROJO": [
        (np.array([0, 50, 60]),   np.array([12, 255, 255])),
        (np.array([165, 50, 60]), np.array([180, 255, 255])),
    ],
    "AZUL": [
        (np.array([100, 120, 70]), np.array([130, 255, 255])),
    ],
}
COLOR_DIBUJO = {"ROJO": (0, 0, 255), "AZUL": (255, 0, 0)}

MODOS = ["AMBOS", "ROJO", "AZUL"]
modo_idx = 0

AREA_MIN, AREA_MAX = 150, 5000
mostrar_mascaras = False

# ==============================================================================
# 1c. PINZA (servo vía Arduino)
# ==============================================================================
ARDUINO_PORT = 'COM7'       # Puerto del Arduino (Administrador de dispositivos)
ARDUINO_BAUD = 115200

# Ángulos del servo (0-180). Calibra con las teclas [O] y [G] y ajusta estos valores.
PINZA_ABIERTA = 60
PINZA_CERRADA = 120

arduino = None
try:
    arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.1)
    time.sleep(2.0)  # El Arduino se reinicia al abrir el puerto serial
    print(f"Arduino (pinza) conectado en {ARDUINO_PORT}.")
except Exception as e:
    print(f"Aviso: Arduino no detectado en {ARDUINO_PORT} ({e}). Pinza desactivada.")

def mover_pinza(angulo):
    angulo = int(max(0, min(180, angulo)))
    if arduino is None:
        print(f"[Pinza simulada] angulo={angulo}")
        return
    arduino.write(f"G{angulo}\n".encode())

# ==============================================================================
# 2. CONEXIÓN DYNAMIXEL Y PERFIL DE VELOCIDAD SUAVE
# ==============================================================================
PORT_U2D2 = 'COM17'
BAUDRATE = 9600

ID_BASE = 1
ID_HOMBRO = 2
ID_CODO = 3

ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED   = 32
ADDR_TORQUE_LIMIT   = 34

VELOCIDAD_SEGURA = 45

portHandler = PortHandler(PORT_U2D2)
packetHandler = PacketHandler(1.0)
CONECTAR_DYNAMIXEL = True

if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
    print("Aviso: U2D2 no detectado. Modo simulación visual activo.")
    CONECTAR_DYNAMIXEL = False
else:
    for m_id in [ID_BASE, ID_HOMBRO, ID_CODO]:
        packetHandler.write1ByteTxRx(portHandler, m_id, ADDR_TORQUE_ENABLE, 1)
        packetHandler.write2ByteTxRx(portHandler, m_id, ADDR_MOVING_SPEED, VELOCIDAD_SEGURA)
        packetHandler.write2ByteTxRx(portHandler, m_id, ADDR_TORQUE_LIMIT, 800)
    print("Dynamixel listos con torque y velocidad suave activa.")

if not CALIBRADO:
    print("\n*** SIN CALIBRAR: el robot NO se moverá. Corre calibrar.py primero. ***\n")

def ir_a_home():
    """Postura HOME guardada en calibracion.json (la que elegiste al calibrar)."""
    if not CONECTAR_DYNAMIXEL or not CALIBRADO:
        return
    print("\n>>> Moviendo suavemente a postura HOME...")
    packetHandler.write2ByteTxRx(portHandler, ID_CODO, ADDR_GOAL_POSITION, int(HOME[2]))
    packetHandler.write2ByteTxRx(portHandler, ID_HOMBRO, ADDR_GOAL_POSITION, int(HOME[1]))
    packetHandler.write2ByteTxRx(portHandler, ID_BASE, ADDR_GOAL_POSITION, int(HOME[0]))

# ==============================================================================
# 3b. FUNCIONES DE CÁMARA Y DETECCIÓN
# ==============================================================================
def listar_camaras(max_idx=6):
    disponibles = []
    print("Buscando cámaras...")
    for i in range(max_idx):
        c = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if c.isOpened():
            ok, f = c.read()
            if ok:
                print(f"  Cámara {i}: OK ({f.shape[1]}x{f.shape[0]})")
                disponibles.append(i)
        c.release()
    if not disponibles:
        print("  No se encontró ninguna cámara.")
    return disponibles

def abrir_camara(idx):
    c = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
    if not c.isOpened():
        print(f"No se pudo abrir la cámara {idx}.")
        return None
    print(f">>> Usando cámara {idx}")
    return c

def mascara_color(hsv, nombre, roi):
    ymin, ymax, xmin, xmax = roi
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in RANGOS_COLOR[nombre]:
        mask |= cv2.inRange(hsv, lo, hi)

    mask_roi = np.zeros_like(mask)
    mask_roi[ymin:ymax, xmin:xmax] = mask[ymin:ymax, xmin:xmax]

    kernel = np.ones((5, 5), np.uint8)
    mask_roi = cv2.morphologyEx(mask_roi, cv2.MORPH_OPEN, kernel)
    mask_roi = cv2.morphologyEx(mask_roi, cv2.MORPH_DILATE, kernel)
    return mask_roi

def detectar_color(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidatos = [c for c in contours if AREA_MIN < cv2.contourArea(c) < AREA_MAX]
    if not candidatos:
        return None
    c = max(candidatos, key=cv2.contourArea)
    M = cv2.moments(c)
    if M["m00"] == 0:
        return None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return c, cx, cy, cv2.contourArea(c)

# ==============================================================================
# 4. BUCLE PRINCIPAL DE VISIÓN Y CONTROL
# ==============================================================================
camaras = listar_camaras()
cap = abrir_camara(CAMARA_INDEX)
if cap is None and camaras:
    CAMARA_INDEX = camaras[0]
    cap = abrir_camara(CAMARA_INDEX)
if cap is None:
    raise SystemExit("No hay cámara disponible.")

ir_a_home()
mover_pinza(PINZA_ABIERTA)

estado_mov = "IDLE"
tiempo_inicio_fase = 0
ticks_grip_actual = None
ticks_approach_actual = None

print("\n================ CONTROLES ================")
print(" [ESPACIO] : Aproximar, agarrar y levantar")
print("    [O]    : Abrir pinza (soltar)")
print("    [G]    : Cerrar pinza")
print("    [T]    : Cambiar objetivo (AMBOS / ROJO / AZUL)")
print("    [C]    : Cambiar de cámara")
print("    [M]    : Ver/ocultar máscaras de color")
print("  [+] [-]  : Subir/bajar altura de agarre (Z_GRIP) 5 mm")
print("    [H]    : Volver a HOME")
print("    [Q]    : Salir del programa")
print("===========================================\n")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w, _ = frame.shape
    centro_cam_x, centro_cam_y = w // 2, h // 2

    roi_ymin, roi_ymax = int(h * 0.12), int(h * 0.70)
    roi_xmin, roi_xmax = int(w * 0.12), int(w * 0.88)
    roi = (roi_ymin, roi_ymax, roi_xmin, roi_xmax)
    cv2.rectangle(frame, (roi_xmin, roi_ymin), (roi_xmax, roi_ymax), (100, 100, 100), 1)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    detecciones = {}
    for nombre in RANGOS_COLOR:
        mask = mascara_color(hsv, nombre, roi)
        if mostrar_mascaras:
            cv2.imshow(f"Mascara {nombre}", mask)

        det = detectar_color(mask)
        if det:
            c, cx, cy, area = det
            detecciones[nombre] = det
            col = COLOR_DIBUJO[nombre]
            cv2.drawContours(frame, [c], -1, col, 2)
            cv2.circle(frame, (cx, cy), 5, col, -1)
            cv2.putText(frame, nombre, (cx + 8, cy - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)

    modo = MODOS[modo_idx]
    elegibles = {n: d for n, d in detecciones.items() if modo == "AMBOS" or n == modo}

    coords_target = None
    ticks_target = None
    color_target = None

    if elegibles:
        color_target = max(elegibles, key=lambda n: elegibles[n][3])
        _, cx, cy, _ = elegibles[color_target]

        cv2.circle(frame, (cx, cy), 12, (0, 255, 0), 2)

        dx_px = cx - centro_cam_x
        dy_px = centro_cam_y - cy

        x_real = DIST_BASE_CAMARA + (dy_px * MM_PER_PIXEL)
        y_real = (-dx_px if INVERTIR_Y else dx_px) * MM_PER_PIXEL
        coords_target = (x_real, y_real)

        ticks_target, status = calcular_ik(x_real, y_real, Z_GRIP)

        color_txt = (0, 255, 0) if status == "OK" else (0, 0, 255)
        cv2.putText(frame, f"{color_target}: X={x_real:.1f} Y={y_real:.1f} mm | {status}",
                    (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_txt, 2)
        if ticks_target:
            cv2.putText(frame, f"Ticks: Base={ticks_target[0]} Hombro={ticks_target[1]} Codo={ticks_target[2]}",
                        (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

            # Comprobación con la cinemática directa: dónde quedaría la punta con esos ticks.
            # Si el error es grande, algún límite de calibracion.json recortó el movimiento.
            fk_x, fk_y, fk_z = calcular_fk(*ticks_target)
            err = math.dist((x_real, y_real, Z_GRIP), (fk_x, fk_y, fk_z))
            col_fk = (0, 255, 0) if err < 10 else (0, 165, 255)
            cv2.putText(frame, f"FK: X={fk_x:.0f} Y={fk_y:.0f} Z={fk_z:.0f} | error={err:.1f} mm",
                        (20, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col_fk, 2)

    cv2.putText(frame, f"Objetivo: {modo} | Camara: {CAMARA_INDEX} | Z_GRIP: {Z_GRIP:.0f}",
                (20, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    if not CALIBRADO:
        cv2.putText(frame, "SIN CALIBRAR - robot bloqueado", (20, 155),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # Máquina de estados no bloqueante: APPROACH -> DESCEND -> GRIP -> LIFT -> IDLE
    tiempo_actual = time.time()
    if estado_mov == "APPROACH":
        cv2.putText(frame, "Fase: Aproximacion aerea...", (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        if tiempo_actual - tiempo_inicio_fase > 1.8:
            print(f"2. Bajando al objeto -> Hombro: {ticks_grip_actual[1]}, Codo: {ticks_grip_actual[2]}")
            packetHandler.write2ByteTxRx(portHandler, ID_HOMBRO, ADDR_GOAL_POSITION, ticks_grip_actual[1])
            packetHandler.write2ByteTxRx(portHandler, ID_CODO, ADDR_GOAL_POSITION, ticks_grip_actual[2])
            estado_mov = "DESCEND"
            tiempo_inicio_fase = tiempo_actual

    elif estado_mov == "DESCEND":
        cv2.putText(frame, "Fase: Posicion de agarre", (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        if tiempo_actual - tiempo_inicio_fase > 1.6:
            print("3. Cerrando pinza...")
            mover_pinza(PINZA_CERRADA)
            estado_mov = "GRIP"
            tiempo_inicio_fase = tiempo_actual

    elif estado_mov == "GRIP":
        cv2.putText(frame, "Fase: Cerrando pinza", (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
        if tiempo_actual - tiempo_inicio_fase > 1.2:
            print("4. Levantando objeto...")
            packetHandler.write2ByteTxRx(portHandler, ID_HOMBRO, ADDR_GOAL_POSITION, ticks_approach_actual[1])
            packetHandler.write2ByteTxRx(portHandler, ID_CODO, ADDR_GOAL_POSITION, ticks_approach_actual[2])
            estado_mov = "LIFT"
            tiempo_inicio_fase = tiempo_actual

    elif estado_mov == "LIFT":
        cv2.putText(frame, "Fase: Levantando", (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        if tiempo_actual - tiempo_inicio_fase > 1.8:
            print(">>> Secuencia completada. Pulsa [O] para soltar o [H] para HOME.")
            estado_mov = "IDLE"

    cv2.drawMarker(frame, (centro_cam_x, centro_cam_y), (255, 0, 0), cv2.MARKER_CROSS, 20, 1)
    cv2.imshow("Deteccion Rojo / Azul", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('h'):
        ir_a_home()
        estado_mov = "IDLE"
    elif key == ord('o'):
        mover_pinza(PINZA_ABIERTA)
    elif key == ord('g'):
        mover_pinza(PINZA_CERRADA)
    elif key == ord('t'):
        modo_idx = (modo_idx + 1) % len(MODOS)
        print(f">>> Objetivo: {MODOS[modo_idx]}")
    elif key == ord('m'):
        mostrar_mascaras = not mostrar_mascaras
        if not mostrar_mascaras:
            for n in RANGOS_COLOR:
                try:
                    cv2.destroyWindow(f"Mascara {n}")
                except cv2.error:
                    pass
    elif key == ord('+') or key == ord('='):
        Z_GRIP += 5.0
        print(f">>> Z_GRIP = {Z_GRIP}")
    elif key == ord('-'):
        Z_GRIP -= 5.0
        print(f">>> Z_GRIP = {Z_GRIP}")
    elif key == ord('c'):
        camaras = listar_camaras()
        if camaras:
            siguiente = camaras[(camaras.index(CAMARA_INDEX) + 1) % len(camaras)] \
                if CAMARA_INDEX in camaras else camaras[0]
            nueva = abrir_camara(siguiente)
            if nueva is not None:
                cap.release()
                cap = nueva
                CAMARA_INDEX = siguiente
    elif key == ord(' ') and coords_target and CONECTAR_DYNAMIXEL and CALIBRADO and estado_mov == "IDLE":
        x_obj, y_obj = coords_target
        ticks_approach, st_app = calcular_ik(x_obj, y_obj, Z_APPROACH)
        ticks_grip, st_grip = calcular_ik(x_obj, y_obj, Z_GRIP)

        if st_app == "OK" and st_grip == "OK":
            print(f"\n1. Aproximación aérea ({color_target}) -> {ticks_approach}")
            mover_pinza(PINZA_ABIERTA)
            packetHandler.write2ByteTxRx(portHandler, ID_BASE, ADDR_GOAL_POSITION, ticks_approach[0])
            packetHandler.write2ByteTxRx(portHandler, ID_HOMBRO, ADDR_GOAL_POSITION, ticks_approach[1])
            packetHandler.write2ByteTxRx(portHandler, ID_CODO, ADDR_GOAL_POSITION, ticks_approach[2])
            ticks_approach_actual = ticks_approach
            ticks_grip_actual = ticks_grip
            estado_mov = "APPROACH"
            tiempo_inicio_fase = time.time()
        else:
            print("Posición fuera de alcance de la cinemática.")

cap.release()
cv2.destroyAllWindows()
if CONECTAR_DYNAMIXEL:
    ir_a_home()
    portHandler.closePort()
if arduino is not None:
    arduino.close()
print("Programa finalizado.")
