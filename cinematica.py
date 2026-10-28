# ==============================================================================
# cinematica.py - Cinemática del brazo de 3 Dynamixel (Base + Hombro + Codo)
# ==============================================================================
# UNIDADES DE MEDIDA UTILIZADAS EN ESTE ARCHIVO:
# - Posiciones espaciales (x, y, z, R, D, L1, L2, Z_HOMBRO): MILÍMETROS (mm)
# - Ángulos articulares (theta_base, theta_hombro, theta_codo, alpha, beta): GRADOS (°)
# - Posiciones de los motores: TICKS (resolución interna del motor servo)
# ==============================================================================

# POR QUÉ EXISTE ESTE ARCHIVO:
# La cámara entrega coordenadas en el espacio cartesiano de la mesa (en mm), 
# pero los motores solo entienden posiciones discretas en "ticks". 
# Este script actúa como el puente matemático bidireccional:
#   * CINEMÁTICA DIRECTA (FK): Convierte ángulos/ticks -> coordenadas (x, y, z) en mm.
#   * CINEMÁTICA INVERSA (IK): Convierte coordenadas (x, y, z) en mm -> ángulos/ticks.

# MODELO GEOMÉTRICO:
# La base rota en el plano horizontal (eje Z). Una vez orientada, el hombro y el codo 
# forman un brazo plano vertical de dos eslabones:
#          codo
#           o---- L2 (188.1 mm) ----> punta (R, z)
#          /
#  L1     / (106.4 mm)
#        /
#       o  hombro (R=0, z=84.7 mm desde la mesa)

import json
import math
import os

_DIR = os.path.dirname(os.path.abspath(__file__))
ARCHIVO_CAL = os.path.join(_DIR, "calibracion.json")

# ------------------------------------------------------------------------------
# Parámetros físicos reales medidos eje a eje (en MILÍMETROS - mm)
# ------------------------------------------------------------------------------
L1 = 106.4            # Distancia en mm: eje del hombro -> eje del codo
L2 = 188.1            # Distancia en mm: eje del codo -> punto de agarre de la pinza
Z_HOMBRO = 84.7       # Altura en mm: superficie de la mesa -> eje del hombro

# ------------------------------------------------------------------------------
# Calibración de motores (Relación Ticks <-> Grados)
# ------------------------------------------------------------------------------
_TPG = 1 / 0.293      # Ticks por grado (escala del motor)
CAL = {
    "base":   {"cero": 512.0,  "signo": -1, "tpg": _TPG, "min": 100, "max": 920},
    "hombro": {"cero": 850 - 90 * _TPG, "signo": +1, "tpg": _TPG, "min": 420, "max": 850},
    "codo":   {"cero": 450 - 180 * _TPG, "signo": +1, "tpg": _TPG, "min": 220, "max": 480},
    "home": [512, 850, 450],
}
CALIBRADO = False
if os.path.exists(ARCHIVO_CAL):
    with open(ARCHIVO_CAL) as f:
        CAL.update(json.load(f))
    CALIBRADO = True
else:
    print("AVISO: no existe calibracion.json -> usando valores supuestos. Correr calibrar.py")

HOME = tuple(CAL["home"])


def ang_a_tick(art, ang_deg):
    """Convierte un ángulo en GRADOS (°) a TICKS del motor correspondiente."""
    c = CAL[art]
    return c["cero"] + c["signo"] * c["tpg"] * ang_deg


def tick_a_ang(art, tick):
    """Convierte los TICKS del motor de regreso a GRADOS (°)."""
    c = CAL[art]
    return (tick - c["cero"]) / (c["signo"] * c["tpg"])


def _limitar(art, tick):
    """Asegura que el valor en ticks se mantenga dentro de los límites seguros."""
    return int(max(CAL[art]["min"], min(CAL[art]["max"], round(tick))))


# ==============================================================================
# CINEMÁTICA INVERSA (IK): De Milímetros (x, y, z) a Ticks de motores
# ==============================================================================
def calcular_ik(x_mm, y_mm, z_mm, devolver_pasos=False):
    """
    Entrada: Coordenadas espaciales en MILÍMETROS (x_mm, y_mm, z_mm).
    Salida: Ticks calculados para los motores y estado de alcance.
    """
    # Paso 1: Orientación de la base en GRADOS (°)
    # Se usa atan2 para calcular el ángulo horizontal hacia el punto (x, y) en mm.
    theta_base = math.degrees(math.atan2(y_mm, x_mm))

    # Paso 2: Reducción 3D a 2D (Cálculo de distancias en MILÍMETROS - mm)
    # R: Alcance horizontal en mm.
    # dz: Desnivel vertical en mm respecto al eje del hombro.
    # D: Distancia directa en línea recta (en mm) desde el hombro hasta la punta.
    R = math.sqrt(x_mm**2 + y_mm**2)
    dz = z_mm - Z_HOMBRO
    D = math.sqrt(R**2 + dz**2)

    # Validación de alcance físico basada en las longitudes L1 y L2 en mm.
    if D > (L1 + L2) or D < abs(L1 - L2):
        if devolver_pasos:
            return None, "Fuera de alcance", {"R": R, "dz": dz, "D": D}
        return None, "Fuera de alcance"

    # Paso 3: Ángulo del Codo en GRADOS (°) usando Ley de Cosenos con L1 y L2 (en mm).
    cos_codo = max(-1.0, min(1.0, (D**2 - L1**2 - L2**2) / (2.0 * L1 * L2)))
    theta_codo = math.degrees(math.acos(cos_codo))

    # Paso 4: Ángulo del Hombro en GRADOS (°) sumando alpha y beta.
    alpha = math.atan2(dz, R)  # Inclinación en radianes de la línea diagonal D
    cos_beta = max(-1.0, min(1.0, (D**2 + L1**2 - L2**2) / (2.0 * L1 * D)))
    beta = math.acos(cos_beta)
    theta_hombro = math.degrees(alpha + beta)

    # Paso 5: Conversión final de GRADOS (°) a TICKS mediante calibración.
    crudo = (ang_a_tick("base", theta_base),
             ang_a_tick("hombro", theta_hombro),
             ang_a_tick("codo", theta_codo))

    # Paso 6: Aplicación de límites físicos de seguridad en ticks.
    ticks = (_limitar("base", crudo[0]), _limitar("hombro", crudo[1]),
             _limitar("codo", crudo[2]))
    
    if not devolver_pasos:
        return ticks, "OK"
    
    sin_lim = tuple(int(round(v)) for v in crudo)
    return ticks, "OK", {
        "theta_base": theta_base, "R": R, "dz": dz, "D": D,
        "cos_codo": cos_codo, "theta_codo": theta_codo,
        "alpha": math.degrees(alpha), "cos_beta": cos_beta,
        "beta": math.degrees(beta), "theta_hombro": theta_hombro,
        "ticks_sin_limites": sin_lim, "recortado": sin_lim != ticks,
    }


# ==============================================================================
# CINEMÁTICA DIRECTA (FK): De Ticks a Milímetros (x, y, z)
# ==============================================================================
def calcular_fk_puntos(tick_base, tick_hombro, tick_codo):
    """
    Convierte ticks de motores a ángulos en GRADOS, y luego aplica trigonometría 
    con L1, L2 y Z_HOMBRO (en MILÍMETROS) para obtener las coordenadas espaciales.
    """
    # Conversión de ticks a radianes/grados
    th_b = math.radians(tick_a_ang("base", tick_base))
    th_h = math.radians(tick_a_ang("hombro", tick_hombro))
    th_c = math.radians(tick_a_ang("codo", tick_codo))

    # Cálculo geométrico de la posición de la punta en MILÍMETROS (mm)
    phi = th_h - th_c
    R_codo = L1 * math.cos(th_h)                  # Alcance horizontal del codo (mm)
    z_codo = Z_HOMBRO + L1 * math.sin(th_h)       # Altura vertical del codo (mm)
    R_punta = R_codo + L2 * math.cos(phi)         # Alcance horizontal total de la punta (mm)
    z_punta = z_codo + L2 * math.sin(phi)         # Altura vertical total de la punta (mm)

    # Proyección tridimensional cartesiana en MILÍMETROS (x, y, z)
    def a3d(R, z):
        return (R * math.cos(th_b), R * math.sin(th_b), z)

    return {"hombro": (0.0, 0.0, Z_HOMBRO), "codo": a3d(R_codo, z_codo),
            "punta": a3d(R_punta, z_punta),
            "R_codo": R_codo, "z_codo": z_codo, "R_punta": R_punta, "z_punta": z_punta,
            "th_b": math.degrees(th_b), "th_h": math.degrees(th_h),
            "th_c": math.degrees(th_c), "phi": math.degrees(phi)}


def calcular_fk(tick_base, tick_hombro, tick_codo):
    """
    Devuelve únicamente la coordenada (x, y, z) en MILÍMETROS de la punta del brazo.
    Se utiliza para verificar la precisión de la cinemática inversa y en la simulación.
    """
    return calcular_fk_puntos(tick_base, tick_hombro, tick_codo)["punta"]