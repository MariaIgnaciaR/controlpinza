"""
calibrar.py - Calibración guiada del brazo (genera calibracion.json)

QUÉ HACE: suelta un motor a la vez (torque OFF) para que lo muevas A MANO, lee
su posición en vivo y te pide guardarla en poses fáciles de medir. Con 2 poses
de ángulo conocido saca, por articulación:  cero, signo y ticks por grado.

QUÉ NECESITAS: una escuadra (o el borde de una caja), y opcionalmente una app de
inclinómetro/nivel en el celular para medir ángulos. Cierra el programa de
visión (solo un programa puede usar el puerto COM).

ADVERTENCIA: al soltar hombro/codo el brazo puede CAER. Sujétalo con la mano.
"""
import json
import os
import sys
import time
from dynamixel_sdk import PortHandler, PacketHandler

PORT, BAUD = "COM17", 9600
IDS = {"base": 1, "hombro": 2, "codo": 3}
ADDR_MODEL, ADDR_TORQUE, ADDR_GOAL, ADDR_SPEED, ADDR_PRESENT = 0, 24, 30, 32, 36
ARCHIVO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibracion.json")
TPG_AX12 = 1024 / 300.0
MODELOS = {12: "AX-12A", 18: "AX-18A", 24: "RX-24F", 28: "RX-28", 29: "MX-28", 310: "MX-64"}

try:
    import msvcrt          # Windows: detectar ENTER sin bloquear
except ImportError:
    msvcrt = None


def ajustar(t1, a1, t2, a2):
    """Recta tick = cero + signo*tpg*theta a partir de 2 poses (tick, ángulo)."""
    pend = (t2 - t1) / (a2 - a1)            # ticks por grado, con signo
    return {"cero": t1 - pend * a1, "signo": 1 if pend > 0 else -1, "tpg": abs(pend)}


class Robot:
    def __init__(self):
        self.port = PortHandler(PORT)
        self.ph = PacketHandler(1.0)
        if not self.port.openPort() or not self.port.setBaudRate(BAUD):
            sys.exit(f"No se pudo abrir {PORT} a {BAUD} baudios (¿otro programa lo usa?).")

    def leer(self, m, addr=ADDR_PRESENT, n=2):
        for _ in range(8):
            f = self.ph.read2ByteTxRx if n == 2 else self.ph.read1ByteTxRx
            v, res, err = f(self.port, m, addr)
            if res == 0 and err == 0:
                return v
            time.sleep(0.02)
        sys.exit(f"El motor ID {m} no responde. Revisa ID, baudrate y alimentación.")

    def torque(self, m, on):
        self.ph.write1ByteTxRx(self.port, m, ADDR_TORQUE, 1 if on else 0)

    def sujetar_todos(self, soltar=()):
        """Todos quedan rígidos en su posición actual, menos los de 'soltar'."""
        for nombre, m in IDS.items():
            p = self.leer(m)
            self.ph.write2ByteTxRx(self.port, m, ADDR_SPEED, 40)
            self.ph.write2ByteTxRx(self.port, m, ADDR_GOAL, p)
            self.torque(m, nombre not in soltar)

    def esperar(self, nombres, texto):
        """Muestra los ticks en vivo hasta que pulses ENTER; devuelve los ticks."""
        print(f"\n>> {texto}")
        ids = [IDS[n] for n in nombres]
        if msvcrt:
            print("   Mueve a mano y pulsa ENTER para guardar.")
            while True:
                vals = [self.leer(m) for m in ids]
                print("\r   " + "  ".join(f"{n}={v:4d}" for n, v in zip(nombres, vals)) + "   ",
                      end="", flush=True)
                if msvcrt.kbhit() and msvcrt.getwch() in ("\r", "\n"):
                    break
                time.sleep(0.05)
            print()
        else:
            input("   Mueve a mano y pulsa ENTER para guardar... ")
        vals = [[self.leer(m) for m in ids] for _ in range(5)]
        return [round(sum(c) / 5) for c in zip(*vals)]


def pedir_angulo(texto, defecto):
    r = input(f"   {texto} [{defecto}]: ").strip().replace(",", ".")
    return float(r) if r else float(defecto)


def articulacion(rob, nombre, extremos, poses):
    print(f"\n{'=' * 60}\nARTICULACIÓN: {nombre.upper()} (ID {IDS[nombre]})\n{'=' * 60}")
    rob.sujetar_todos(soltar=(nombre,))
    print("   Torque OFF en esta articulación: SUJETA el brazo.")
    t_ext = [rob.esperar([nombre], e)[0] for e in extremos]
    medidas = []
    for texto, ang_def in poses:
        t = rob.esperar([nombre], texto)[0]
        a = pedir_angulo("¿Cuántos grados mide esa pose?", ang_def)
        medidas.append((t, a))
    (t1, a1), (t2, a2) = medidas
    if a1 == a2 or t1 == t2:
        sys.exit("Las dos poses son iguales: repite esta articulación.")
    aj = ajustar(t1, a1, t2, a2)
    esperado = TPG_AX12
    if abs(aj["tpg"] - esperado) / esperado > 0.10:
        print(f"   ¡OJO! Medí {aj['tpg']:.2f} ticks/° y un AX-12 da {esperado:.2f}. "
              "Revisa los ángulos que escribiste, o si el motor es otro modelo (MX = 11.38).")
    aj["min"], aj["max"] = min(t_ext), max(t_ext)
    print(f"   -> cero={aj['cero']:.1f}  signo={aj['signo']:+d}  tpg={aj['tpg']:.3f}  "
          f"límites=[{aj['min']}, {aj['max']}]")
    rob.sujetar_todos()
    return aj


def main():
    rob = Robot()
    print("Modelos detectados:", {n: MODELOS.get(rob.leer(m, ADDR_MODEL), rob.leer(m, ADDR_MODEL))
                                   for n, m in IDS.items()})
    print(__doc__)
    input("ENTER para empezar...")
    cal = {}
    cal["base"] = articulacion(
        rob, "base",
        ["Gira la base hasta el extremo SEGURO de la IZQUIERDA",
         "Gira la base hasta el extremo SEGURO de la DERECHA"],
        [("Brazo apuntando RECTO AL FRENTE (hacia donde mira la cámara). Eso es 0°", 0),
         ("Brazo girado a la IZQUIERDA (mirando desde la base hacia adelante). "
          "Usa la escuadra: 90° es lo más fácil", 90)])
    cal["hombro"] = articulacion(
        rob, "hombro",
        ["Hombro en el extremo SEGURO de ABAJO (sin tocar la mesa)",
         "Hombro en el extremo SEGURO de ARRIBA"],
        [("Eslabón 1 HORIZONTAL apuntando hacia adelante (0°)", 0),
         ("Eslabón 1 VERTICAL hacia arriba (90°). Déjalo así: el codo se calibra con él vertical", 90)])
    cal["codo"] = articulacion(
        rob, "codo",
        ["Codo en el extremo SEGURO más ESTIRADO",
         "Codo en el extremo SEGURO más PLEGADO"],
        [("Eslabón 2 EN LÍNEA RECTA con el 1 (hacia arriba). Pliegue = 0°", 0),
         ("Eslabón 2 HORIZONTAL hacia adelante (L1 vertical): pliegue = 90°", 90)])
    rob.sujetar_todos(soltar=tuple(IDS))
    print("\nPostura HOME (reposo): SUJETA el brazo (torque OFF en los 3).")
    h = rob.esperar(list(IDS), "Pon el brazo en la postura HOME que quieras")
    cal["home"] = h
    rob.sujetar_todos()
    with open(ARCHIVO, "w") as f:
        json.dump(cal, f, indent=2)
    print(f"\nGuardado en {ARCHIVO}\nSiguiente paso: python probar_punto.py")


if __name__ == "__main__":
    main()
