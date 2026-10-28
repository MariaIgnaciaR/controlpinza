"""
simulacion_brazo.py - Simulación fluida (pygame, 60 fps) del brazo con traslado autónomo por color.   pip install pygame
"""
import math
import sys
import time
import pygame
import cinematica as K

W, H = 1180, 720
Z_APPROACH, Z_GRIP = 150.0, 90.0  # Alturas seguras para evitar colisiones con la mesa
PA, PC = 60, 120
BG, PANEL = (20, 23, 27), (28, 32, 37)
FG, MUT, GRID = (230, 233, 238), (130, 140, 150), (52, 58, 66)
C1, C2, OK, BAD = (91, 163, 255), (255, 138, 92), (76, 214, 138), (235, 80, 80)
RL = pygame.Rect(20, 50, 560, 430)
RT = pygame.Rect(600, 50, 560, 430)
KS = 1.15

est = dict(zip("bse", map(float, K.HOME)), g=float(PA))
obj = {"x": 200.0, "y": 60.0, "z": Z_APPROACH, "ok": True}
anim = {"seg": None, "cola": [], "msg": "Listo"}

# Coordenadas de racks ajustadas para una cinemática fluida y sin cruzar la mesa
RACKS = {
    "rojo":    {"x": 150.0, "y": -70.0, "color": (235, 80, 80), "nombre": "Rack Origen Rojo"},
    "azul":    {"x": 150.0, "y": 70.0,  "color": (91, 163, 255), "nombre": "Rack Origen Azul"},
    "destino": {"x": 180.0, "y": 0.0,   "color": (76, 214, 138), "nombre": "Rack Destino Común"}
}


def aplicar_obj(x, y, z):
    obj.update(x=x, y=y, z=z)
    ticks, _ = K.calcular_ik(x, y, z)
    obj["ok"] = ticks is not None
    if ticks:
        est["b"], est["s"], est["e"] = map(float, ticks)
        anim["msg"] = "Manual / Personalizado"
    else:
        anim["msg"] = "Fuera de alcance"


def obj_desde_fk():
    x, y, z = K.calcular_fk(est["b"], est["s"], est["e"])
    obj.update(x=x, y=y, z=z, ok=True)


def parar():
    anim["seg"], anim["cola"] = None, []


def siguiente():
    if anim["cola"]:
        t, dur, txt = anim["cola"].pop(0)
        anim["seg"] = dict(f=dict(est), t={**est, **t}, d=dur, t0=time.time())
        anim["msg"] = txt
    else:
        anim["seg"], anim["msg"] = None, "Secuencia de traslado completada con éxito"


def ejecutar_traslado(color):
    if color not in RACKS:
        return
    
    origen = RACKS[color]
    destino = RACKS["destino"]
    
    ta_orig, _ = K.calcular_ik(origen["x"], origen["y"], Z_APPROACH)
    tg_orig, _ = K.calcular_ik(origen["x"], origen["y"], Z_GRIP)
    ta_dest, _ = K.calcular_ik(destino["x"], destino["y"], Z_APPROACH)
    tg_dest, _ = K.calcular_ik(destino["x"], destino["y"], Z_GRIP)
    
    if None in (ta_orig, tg_orig, ta_dest, tg_dest):
        anim["msg"] = f"Error: Rack {color} o Destino fuera de alcance"
        return

    P = lambda t: dict(b=t[0], s=t[1], e=t[2])
    
    anim["cola"] = [
        (dict(g=PA), 0.8, f"Abriendo pinza"),
        (dict(P(ta_orig), g=PA), 1.3, f"1. Aproximación aérea a probeta {color}"),
        (dict(P(tg_orig), g=PA), 1.0, f"2. Descenso seguro a probeta {color}"),
        (dict(P(tg_orig), g=PC), 0.8, f"3. Agarrando probeta {color}"),
        (dict(P(ta_orig), g=PC), 1.3, f"4. Levantando probeta {color}"),
        (dict(P(ta_dest), g=PC), 1.5, f"5. Trasladando hacia rack destino"),
        (dict(P(tg_dest), g=PC), 1.0, f"6. Descenso en rack destino"),
        (dict(P(tg_dest), g=PA), 0.8, f"7. Soltando probeta en destino"),
        (dict(P(ta_dest), g=PA), 1.0, f"8. Retirada segura")
    ]
    siguiente()


def secuencia():
    x, y, _ = K.calcular_fk(est["b"], est["s"], est["e"])
    ta, _ = K.calcular_ik(x, y, Z_APPROACH)
    tg, _ = K.calcular_ik(x, y, Z_GRIP)
    if ta is None or tg is None:
        anim["msg"] = "Fuera de alcance"
        return
    P = lambda t: dict(b=t[0], s=t[1], e=t[2])
    anim["cola"] = [(dict(P(ta), g=PA), 1.5, "1. Aproximación aérea"),
                    (P(tg), 1.2, "2. Descenso"),
                    (dict(g=PC), 1.0, "3. Cerrar pinza"),
                    (P(ta), 1.5, "4. Levantar")]
    siguiente()


def paso_anim():
    s = anim["seg"]
    if not s:
        return
    u = min(1.0, (time.time() - s["t0"]) / s["d"])
    e = 2 * u * u if u < .5 else 1 - (-2 * u + 2) ** 2 / 2
    for k in "bseg":
        est[k] = s["f"][k] + (s["t"][k] - s["f"][k]) * e
    obj_desde_fk()
    if u >= 1:
        siguiente()


def arco(cx, cy, r, a0, a1, n=60):
    return [(cx + r * math.cos(a0 + (a1 - a0) * i / n), cy - r * math.sin(a0 + (a1 - a0) * i / n))
            for i in range(n + 1)]


def dibujar(s, fnt, fgr):
    s.fill(BG)
    for r, t in ((RL, "Vista lateral (plano del brazo)"), (RT, "Vista superior (+Y a la izquierda)")):
        pygame.draw.rect(s, PANEL, r, border_radius=6)
        pygame.draw.rect(s, GRID, r, 1, border_radius=6)
        s.blit(fgr.render(t, True, MUT), (r.x, r.y - 22))
    p = K.calcular_fk_puntos(est["b"], est["s"], est["e"])
    x, y, z = p["punta"]
    col_obj = OK if obj["ok"] else BAD
    
    # Lateral
    s.set_clip(RL)
    ox, oy = RL.x + 150, RL.bottom - 40
    SX = lambda R: ox + R * KS
    SY = lambda zz: oy - zz * KS
    pygame.draw.line(s, MUT, (RL.x, oy), (RL.right, oy), 3)
    pygame.draw.rect(s, MUT, (SX(-18), SY(K.Z_HOMBRO), 36 * KS, K.Z_HOMBRO * KS))
    pygame.draw.lines(s, GRID, False, arco(SX(0), SY(K.Z_HOMBRO), (K.L1 + K.L2) * KS, -math.pi / 2, math.pi / 2), 1)
    pygame.draw.line(s, GRID, (SX(0), SY(K.Z_HOMBRO)), (SX(300), SY(K.Z_HOMBRO)), 1)
    Rc, zc, Rp, zp = p["R_codo"], p["z_codo"], p["R_punta"], p["z_punta"]
    pygame.draw.line(s, C1, (SX(0), SY(K.Z_HOMBRO)), (SX(Rc), SY(zc)), 10)
    pygame.draw.line(s, C2, (SX(Rc), SY(zc)), (SX(Rp), SY(zp)), 10)
    for pt in ((0, K.Z_HOMBRO), (Rc, zc)):
        pygame.draw.circle(s, FG, (SX(pt[0]), SY(pt[1])), 7)
    ph = math.radians(p["phi"])
    dx, dy = math.cos(ph), math.sin(ph)
    gap = 6 + (PC - est["g"]) / 60 * 14
    for sg in (1, -1):
        a = (SX(Rp - 20 * dx - dy * gap * sg), SY(zp - 20 * dy + dx * gap * sg))
        b = (SX(Rp - dy * gap * sg), SY(zp + dx * gap * sg))
        pygame.draw.line(s, FG, a, b, 4)
    pygame.draw.circle(s, OK, (SX(Rp), SY(zp)), 5)
    Ro = math.hypot(obj["x"], obj["y"])
    pygame.draw.circle(s, col_obj, (SX(Ro), SY(obj["z"])), 11, 2)
    s.set_clip(None)
    
    # Superior
    s.set_clip(RT)
    cx, cy = RT.x + RT.w // 2, RT.bottom - 40
    R0 = K.L1 + K.L2
    pts = [(cx - R0 * KS * math.sin(u), cy - R0 * KS * math.cos(u))
           for u in [i * math.pi / 60 - math.pi / 2 for i in range(61)]]
    pygame.draw.lines(s, GRID, False, pts, 1)
    pygame.draw.line(s, GRID, (cx, cy), (cx, cy - 330 * KS), 1)
    
    for k, rk in RACKS.items():
        rx_scr = cx - rk["y"] * KS
        ry_scr = cy - rk["x"] * KS
        pygame.draw.circle(s, rk["color"], (rx_scr, ry_scr), 12)
        s.blit(fnt.render(k.upper(), True, FG), (rx_scr - 15, ry_scr - 25))

    tip = (cx - y * KS, cy - x * KS)
    pygame.draw.line(s, C1, (cx, cy), tip, 10)
    pygame.draw.circle(s, FG, (cx, cy), 10)
    pygame.draw.circle(s, OK, tip, 6)
    pygame.draw.circle(s, col_obj, (cx - obj["y"] * KS, cy - obj["x"] * KS), 12, 2)
    s.set_clip(None)
    
    fk = ["CINEMÁTICA DIRECTA (ticks -> x,y,z)",
          f"θbase   = {p['th_b']:.1f}°",
          f"θhombro = {p['th_h']:.1f}°",
          f"θcodo   = {p['th_c']:.1f}°",
          f"R = {p['R_punta']:.1f} mm | z = {p['z_punta']:.1f} mm",
          f"x = {x:.1f} | y = {y:.1f}"]
    
    t, st, d = K.calcular_ik(x, y, z, devolver_pasos=True)
    if t is None:
        ik = [f"CINEMÁTica Inversa", "FUERA DE ALCANCE"]
    else:
        ik = [f"CINEMÁTICA INVERSA (IK Activa)",
              f"θbase = {d['theta_base']:.1f}° | R = {d['R']:.1f}",
              f"θcodo = {d['theta_codo']:.1f}°",
              f"θhombro = {d['theta_hombro']:.1f}°",
              f"Ticks calculados por software"]

    for i, l in enumerate(fk):
        s.blit(fnt.render(l, True, FG if i else C1), (20, 495 + i * 20))
    for i, l in enumerate(ik):
        s.blit(fnt.render(l, True, FG if i else C2), (600, 495 + i * 20))
        
    s.blit(fgr.render(f"Estado Autónomo: {anim['msg']}", True, OK), (20, 628))
    s.blit(fnt.render("[1 / R] Mover Probeta ROJA | [2 / A] Mover Probeta AZUL | [ESPACIO] Secuencia | [ESC] Salir", True, MUT), (20, 690))


def main(frames=None):
    pygame.init()
    s = pygame.display.set_mode((W, H))
    pygame.display.set_caption("Simulación Brazo 3 GDL - Traslado de Probetas Autónomo")
    clk = pygame.time.Clock()
    fnt = pygame.font.SysFont("consolas,couriernew,monospace", 14)
    fgr = pygame.font.SysFont("segoeui,arial,sans", 16, bold=True)
    aplicar_obj(150, -70, Z_APPROACH)
    n = 0
    while frames is None or n < frames:
        n += 1
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                pygame.quit(); return
            if ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_SPACE: secuencia()
                elif ev.key in (pygame.K_1, pygame.K_r):
                    parar(); ejecutar_traslado("rojo")
                elif ev.key in (pygame.K_2, pygame.K_a):
                    parar(); ejecutar_traslado("azul")
                elif ev.key == pygame.K_h:
                    parar(); est.update(zip("bse", map(float, K.HOME))); obj_desde_fk()
                elif ev.key == pygame.K_o: est["g"] = PA
                elif ev.key == pygame.K_g: est["g"] = PC
            if ev.type == pygame.MOUSEWHEEL:
                parar(); aplicar_obj(obj["x"], obj["y"], obj["z"] + 5 * ev.y)
        
        k = pygame.key.get_pressed()
        dk = (k[pygame.K_LEFT] - k[pygame.K_RIGHT], k[pygame.K_UP] - k[pygame.K_DOWN], k[pygame.K_x] - k[pygame.K_z])
        if any(dk):
            parar()
            for key, d, art in zip("bse", dk, ("base", "hombro", "codo")):
                est[key] = max(K.CAL[art]["min"], min(K.CAL[art]["max"], est[key] + 2 * d))
            obj_desde_fk()
            
        if pygame.mouse.get_pressed()[0]:
            mx, my = pygame.mouse.get_pos()
            if RT.collidepoint(mx, my):
                parar(); aplicar_obj((RT.bottom - 40 - my) / KS, (RT.x + RT.w // 2 - mx) / KS, obj["z"])
            elif RL.collidepoint(mx, my):
                parar(); R = (mx - (RL.x + 150)) / KS; zz = (RL.bottom - 40 - my) / KS
                th = math.atan2(obj["y"], obj["x"])
                aplicar_obj(R * math.cos(th), R * math.sin(th), zz)
                
        paso_anim()
        dibujar(s, fnt, fgr)
        pygame.display.flip()
        clk.tick(60)
    return s


if __name__ == "__main__":
    main()