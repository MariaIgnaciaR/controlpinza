import cv2
import numpy as np
import math
import time
from dynamixel_sdk import *

# ==============================================================================
# 1. PARÁMETROS GEOMÉTRICOS DEL ROBOT (mm)
# ==============================================================================
L1 = 106.4
L2 = 188.1
Z_HOMBRO = 84.7
H_OBJETO = 96.1

# Alturas seguras de trabajo
Z_APPROACH = 105.0         # Altura aérea
Z_GRIP = 42.0              # Agarre profundo para alcanzar la pieza
MM_PER_PIXEL = 0.55

# Calibración HSV obtenida de tu barra verde
LOWER_VERDE = np.array([0, 78, 146])
UPPER_VERDE = np.array([180, 173, 255])

# ==============================================================================
# 2. CONEXIÓN Y PERFIL DE VELOCIDAD SUAVE
# ==============================================================================
PORT_U2D2 = 'COM15'
BAUDRATE = 9600

ID_BASE = 1
ID_HOMBRO = 2
ID_CODO = 3

ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED   = 32
ADDR_TORQUE_LIMIT   = 34

VELOCIDAD_SEGURA = 45  # Movimiento controlado sin sacudidas

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

# ==============================================================================
# 3. CINEMÁTICA CON EXTENSIÓN PLENA Y DESCENSO PROFUNDO
# ==============================================================================
def calcular_ik(x_mm, y_mm, z_mm):
    # 1. Base orientada hacia el objeto
    theta_base_rad = math.atan2(y_mm, x_mm)
    theta_base_deg = math.degrees(theta_base_rad)

    R = math.sqrt(x_mm**2 + y_mm**2)
    dz = z_mm - Z_HOMBRO
    D = math.sqrt(R**2 + dz**2)

    if D > (L1 + L2) or D < abs(L1 - L2):
        return None, "Fuera de alcance"

    # Ley de Cosenos
    cos_codo = (D**2 - L1**2 - L2**2) / (2.0 * L1 * L2)
    cos_codo = max(-1.0, min(1.0, cos_codo))
    theta_codo_rad = math.acos(cos_codo)

    alpha = math.atan2(dz, R)
    cos_beta = (D**2 + L1**2 - L2**2) / (2.0 * L1 * D)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)

    theta_hombro_rad = alpha + beta

    tick_base = int(512 - (theta_base_deg / 0.293))

    # 2. Hombro (ID 2): 850 es vertical hacia arriba.
    # Liberado hasta 420 ticks para estirarse al frente
    deg_hombro = math.degrees(theta_hombro_rad)
    tick_hombro = int(850 - ((90.0 - deg_hombro) / 0.293))
    tick_hombro = max(420, min(850, tick_hombro))

    # 3. Codo (ID 3): 450 vertical arriba, 200 perpendicular.
    # Permitido hasta 165 ticks para agachar la pinza sin chocar con la mesa
    deg_codo = math.degrees(theta_codo_rad)
    tick_codo = int(450 - ((180.0 - deg_codo) / 0.293))

    # Límites físicos seguros
    tick_base = max(100, min(920, tick_base))
    tick_codo = max(165, min(480, tick_codo))

    return (tick_base, tick_hombro, tick_codo), "OK"

def ir_a_home():
    """Postura HOME: Hombro erguido a 90° (850) y Codo recogido arriba (450)."""
    if not CONECTAR_DYNAMIXEL:
        return
    print("\n>>> Moviendo suavemente a postura HOME...")
    packetHandler.write2ByteTxRx(portHandler, ID_CODO, ADDR_GOAL_POSITION, 450)
    packetHandler.write2ByteTxRx(portHandler, ID_HOMBRO, ADDR_GOAL_POSITION, 850)
    packetHandler.write2ByteTxRx(portHandler, ID_BASE, ADDR_GOAL_POSITION, 512)

# ==============================================================================
# 4. BUCLE PRINCIPAL DE VISIÓN Y CONTROL
# ==============================================================================
cap = cv2.VideoCapture(1)
if not cap.isOpened():
    cap = cv2.VideoCapture(0)

ir_a_home()

estado_mov = "IDLE"
tiempo_inicio_fase = 0
ticks_grip_actual = None

print("\n================ CONTROLES ================")
print(" [ESPACIO] : Ejecutar aproximación y agarre completo")
print("    [H]    : Volver a HOME (90° vertical)")
print("    [Q]    : Salir del programa")
print("===========================================\n")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w, _ = frame.shape
    centro_cam_x, centro_cam_y = w // 2, h // 2

    # Zona segura ROI en la mesa
    roi_ymin, roi_ymax = int(h * 0.12), int(h * 0.70)
    roi_xmin, roi_xmax = int(w * 0.12), int(w * 0.88)
    cv2.rectangle(frame, (roi_xmin, roi_ymin), (roi_xmax, roi_ymax), (100, 100, 100), 1)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_VERDE, UPPER_VERDE)

    mask_roi = np.zeros_like(mask)
    mask_roi[roi_ymin:roi_ymax, roi_xmin:roi_xmax] = mask[roi_ymin:roi_ymax, roi_xmin:roi_xmax]

    kernel = np.ones((5, 5), np.uint8)
    mask_roi = cv2.morphologyEx(mask_roi, cv2.MORPH_OPEN, kernel)
    mask_roi = cv2.morphologyEx(mask_roi, cv2.MORPH_DILATE, kernel)

    contours, _ = cv2.findContours(mask_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    coords_target = None
    ticks_target = None

    if contours:
        candidatos = [cnt for cnt in contours if 80 < cv2.contourArea(cnt) < 5000]
        if candidatos:
            c = max(candidatos, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                cv2.circle(frame, (cx, cy), 6, (0, 255, 0), -1)
                cv2.drawContours(frame, [c], -1, (0, 255, 255), 2)

                dx_px = cx - centro_cam_x
                dy_px = centro_cam_y - cy

                x_real = 180.0 + (dy_px * MM_PER_PIXEL)
                y_real = dx_px * MM_PER_PIXEL
                coords_target = (x_real, y_real)

                ticks_target, status = calcular_ik(x_real, y_real, Z_GRIP)

                color_txt = (0, 255, 0) if status == "OK" else (0, 0, 255)
                cv2.putText(frame, f"Obj: X={x_real:.1f} Y={y_real:.1f} mm | {status}",
                            (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_txt, 2)
                if ticks_target:
                    cv2.putText(frame, f"Ticks: Base={ticks_target[0]} Hombro={ticks_target[1]} Codo={ticks_target[2]}",
                                (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    # Máquina de estados no bloqueante
    tiempo_actual = time.time()
    if estado_mov == "APPROACH":
        cv2.putText(frame, "Fase: Aproximacion aerea...", (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        if tiempo_actual - tiempo_inicio_fase > 1.8:
            print(f"2. Extendiéndose a la barra -> Hombro: {ticks_grip_actual[1]}, Codo: {ticks_grip_actual[2]}")
            packetHandler.write2ByteTxRx(portHandler, ID_HOMBRO, ADDR_GOAL_POSITION, ticks_grip_actual[1])
            packetHandler.write2ByteTxRx(portHandler, ID_CODO, ADDR_GOAL_POSITION, ticks_grip_actual[2])
            estado_mov = "DESCEND"
            tiempo_inicio_fase = tiempo_actual

    elif estado_mov == "DESCEND":
        cv2.putText(frame, "Fase: Posicion de agarre", (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        if tiempo_actual - tiempo_inicio_fase > 1.6:
            print(">>> Trayectoria completada con éxito.")
            estado_mov = "IDLE"

    cv2.drawMarker(frame, (centro_cam_x, centro_cam_y), (255, 0, 0), cv2.MARKER_CROSS, 20, 1)
    cv2.imshow("Deteccion Barra Verde", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('h'):
        ir_a_home()
        estado_mov = "IDLE"
    elif key == ord(' ') and coords_target and CONECTAR_DYNAMIXEL and estado_mov == "IDLE":
        x_obj, y_obj = coords_target
        ticks_approach, st_app = calcular_ik(x_obj, y_obj, Z_APPROACH)
        ticks_grip, st_grip = calcular_ik(x_obj, y_obj, Z_GRIP)

        if st_app == "OK" and st_grip == "OK":
            print(f"\n1. Aproximación aérea -> {ticks_approach}")
            packetHandler.write2ByteTxRx(portHandler, ID_BASE, ADDR_GOAL_POSITION, ticks_approach[0])
            packetHandler.write2ByteTxRx(portHandler, ID_HOMBRO, ADDR_GOAL_POSITION, ticks_approach[1])
            packetHandler.write2ByteTxRx(portHandler, ID_CODO, ADDR_GOAL_POSITION, ticks_approach[2])
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
print("Programa finalizado.")