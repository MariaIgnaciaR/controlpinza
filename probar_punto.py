"""
probar_punto.py - Verifica la calibración en el robot REAL.

Le das un punto (x, y, z) en mm; el programa calcula la IK, te dice dónde
debería quedar la punta (FK) y mueve el robot LENTO. Tú mides con la regla
dónde quedó realmente. Si coincide (±5 mm), la cinemática está bien.

Ejemplos:  200 0 105   (adelante, en el aire)      home   (postura HOME)
Empieza SIEMPRE con z alto (100-150) y baja de a poco.
"""
import math
import sys
from dynamixel_sdk import PortHandler, PacketHandler
import cinematica as K

PORT, BAUD = "COM17", 9600
IDS = (1, 2, 3)   # base, hombro, codo
port = PortHandler(PORT)
ph = PacketHandler(1.0)
if not port.openPort() or not port.setBaudRate(BAUD):
    sys.exit("No se pudo abrir el puerto.")
if not K.CALIBRADO:
    print("¡Falta calibracion.json! Corre calibrar.py primero.")


def mover(ticks):
    for m, t in zip(IDS, ticks):
        ph.write1ByteTxRx(port, m, 24, 1)
        ph.write2ByteTxRx(port, m, 32, 40)      # velocidad lenta
        ph.write2ByteTxRx(port, m, 30, int(t))


while True:
    txt = input("\nx y z en mm (o 'home', 'q'): ").strip().lower()
    if txt == "q":
        break
    if txt == "home":
        mover(K.HOME)
        continue
    try:
        x, y, z = (float(v) for v in txt.replace(",", ".").split())
    except ValueError:
        print("Escribe tres números, ej: 200 0 105")
        continue
    ticks, st, d = K.calcular_ik(x, y, z, devolver_pasos=True)
    if ticks is None:
        print(f"Fuera de alcance (D={d['D']:.1f} mm; máx {K.L1 + K.L2:.1f}).")
        continue
    fx, fy, fz = K.calcular_fk(*ticks)
    err = math.dist((x, y, z), (fx, fy, fz))
    print(f"Ticks base/hombro/codo = {ticks}")
    print(f"θbase={d['theta_base']:.1f}°  θhombro={d['theta_hombro']:.1f}°  θcodo={d['theta_codo']:.1f}°")
    print(f"FK: la punta quedaría en ({fx:.0f}, {fy:.0f}, {fz:.0f})  error={err:.1f} mm")
    if d["recortado"]:
        print("¡AVISO! Un límite recortó el movimiento: la punta no llegará a ese punto.")
    if input("¿Mover? (s/n): ").strip().lower() == "s":
        mover(ticks)
        print("Mide con la regla dónde quedó la punta y compara.")
