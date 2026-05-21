#!/usr/bin/env python3
# coding: utf-8
"""
P4 - Evasion de Colisiones (Problema Abierto)
MyCobot 280 - 6 DOF

Estrategia combinada:
  1. Limites conservadores por joint (mas estrechos que los fisicos)
  2. Verificacion de altura minima Z via FK simplificada
  3. Waypoint seguro de clearance entre cualquier movimiento no trivial

Escenarios de colision identificados:
  (a) Colision con la mesa  - Z demasiado baja
  (b) Auto-colision         - J2/J3 en angulos extremos opuestos
  (c) Colision con objetos  - movimiento lateral sin clearance
"""

import time
import numpy as np
from math import cos, sin, radians

# -----------------------------------------------------------------------
# Escenarios de colision identificados en el laboratorio
# -----------------------------------------------------------------------
COLLISION_SCENARIOS = {
    "colision_mesa": {
        "descripcion": "El extremo o eslabon impacta la superficie de la mesa.",
        "causa":       "Altura Z calculada por FK resulta menor que Z_MIN_SAFE.",
        "prevencion":  "Verificacion de Z minima antes de cada send_angles().",
    },
    "auto_colision": {
        "descripcion": "El codo (J3) golpea el eslabon del hombro (J2) o el cuerpo del robot.",
        "causa":       "J2 y J3 en angulos extremos que acortan el eslabon efectivo.",
        "prevencion":  "Limites conservadores: J2 en [-110, 70], J3 en [-120, 120].",
    },
    "colision_objetos": {
        "descripcion": "El brazo choca con la camara, zona de deposito u obstaculos fijos.",
        "causa":       "Movimiento directo al destino sin trayectoria intermedia segura.",
        "prevencion":  "Waypoint seguro [0, 0, -90, 90, 0, -45] con clearance garantizado.",
    },
}

# -----------------------------------------------------------------------
# Limites conservadores por joint (grados)
# Mas estrechos que los limites fisicos del hardware para mayor seguridad.
# -----------------------------------------------------------------------
CONSERVATIVE_JOINT_LIMITS = {
    0: (-160, 160),   # J1 Base      (fisico: -168, 168)
    1: (-110,  70),   # J2 Hombro    (fisico: -135,  90)
    2: (-120, 120),   # J3 Codo      (fisico: -150, 150)
    3: (-130, 130),   # J4 Muneca1   (fisico: -145, 145)
    4: (-150, 150),   # J5 Muneca2   (fisico: -165, 165)
    5: (-175, 175),   # J6 Gripper   (fisico: -180, 180)
}

Z_MIN_SAFE = 60.0   # mm - altura minima del extremo sobre la mesa

# Waypoint de clearance garantizado: brazo en posicion vertical con codo plegado
SAFE_WAYPOINT_ANGLES = [0, 0, -90, 90, 0, -45]


def _forward_kinematics_simplified(angles):
    """
    FK simplificada de 3 joints (modelo planar) para estimar Z del extremo.
    Suficiente para la verificacion de colision con la mesa.
    """
    theta1 = radians(angles[0])
    theta2 = radians(angles[1])
    theta3 = radians(angles[2])
    L1, L2, L3 = 131.0, 110.0, 96.0
    r = L2 * cos(theta2) + L3 * cos(theta2 + theta3)
    x = r * cos(theta1)
    y = r * sin(theta1)
    z = L1 + L2 * sin(theta2) + L3 * sin(theta2 + theta3)
    T = np.eye(4)
    T[0, 3], T[1, 3], T[2, 3] = x, y, z
    return T


class CollisionChecker:
    """
    Verifica colisiones potenciales antes de ejecutar movimientos.

    Uso:
        checker = CollisionChecker(mc=mc)
        if checker.check_collision(target_angles):
            mc.send_angles(target_angles, speed)
        else:
            checker.safe_move(target_angles, speed)
    """

    def __init__(self, mc=None):
        self.mc = mc
        self.joint_limits = CONSERVATIVE_JOINT_LIMITS
        self.z_min = Z_MIN_SAFE

    # ------------------------------------------------------------------
    # Verificacion de limites articulares
    # ------------------------------------------------------------------
    def check_joint_limits(self, angles):
        """
        Verifica que todos los angulos esten dentro de CONSERVATIVE_JOINT_LIMITS.

        Retorna
        -------
        (ok: bool, msg: str)
        """
        for i, angle in enumerate(angles):
            lo, hi = self.joint_limits[i]
            if not (lo <= angle <= hi):
                return False, f"J{i+1}={angle:.1f} fuera de [{lo}, {hi}]"
        return True, "OK"

    # ------------------------------------------------------------------
    # Verificacion de altura minima
    # ------------------------------------------------------------------
    def check_min_height(self, angles):
        """
        Calcula Z del extremo via FK simplificada y verifica Z >= Z_MIN_SAFE.

        Retorna
        -------
        (ok: bool, z_value: float, msg: str)
        """
        T = _forward_kinematics_simplified(angles)
        z = T[2, 3]
        if z < self.z_min:
            return False, z, f"Z={z:.1f} mm < Z_min={self.z_min} mm (PELIGROSA)"
        return True, z, f"Z={z:.1f} mm >= Z_min={self.z_min} mm (SEGURA)"

    # ------------------------------------------------------------------
    # Verificacion combinada (True = seguro)
    # ------------------------------------------------------------------
    def check_collision(self, angles):
        """
        Ejecuta ambas verificaciones. Retorna True si la configuracion es segura.
        """
        ok_limits, _ = self.check_joint_limits(angles)
        if not ok_limits:
            return False
        ok_height, _, _ = self.check_min_height(angles)
        return ok_height

    # ------------------------------------------------------------------
    # Movimiento seguro via waypoint de clearance
    # ------------------------------------------------------------------
    def safe_move(self, target_angles, speed=20):
        """
        Mueve el robot al destino pasando por SAFE_WAYPOINT_ANGLES.
        Rechaza el movimiento si el destino viola limites o altura minima.

        Retorna True si el movimiento se ejecuto, False si fue rechazado.
        """
        ok_l, msg_l = self.check_joint_limits(target_angles)
        if not ok_l:
            print(f"[P4] Limite articular: {msg_l}")
            return False

        ok_z, z_val, msg_z = self.check_min_height(target_angles)
        if not ok_z:
            print(f"[P4] Altura insegura: {msg_z}")
            return False

        print(f"[P4] Altura verificada: {msg_z}")

        if self.mc is not None:
            print(f"[P4] Moviendo a waypoint seguro {SAFE_WAYPOINT_ANGLES}...")
            self.mc.send_angles(SAFE_WAYPOINT_ANGLES, speed)
            time.sleep(3.0)
            print(f"[P4] Moviendo a destino {[round(a, 1) for a in target_angles]}...")
            self.mc.send_angles(target_angles, speed)
            time.sleep(4.0)
        return True


if __name__ == "__main__":
    print("P4 - Evasion de Colisiones - MyCobot 280\n")

    print("Escenarios identificados:")
    for k, v in COLLISION_SCENARIOS.items():
        print(f"  [{k}]")
        print(f"    Descripcion: {v['descripcion']}")
        print(f"    Causa:       {v['causa']}")
        print(f"    Prevencion:  {v['prevencion']}")

    checker = CollisionChecker()

    tests = [
        ([0, 0, 0, 0, 0, -45],   "HOME segura"),
        ([30, 0, 0, -45, 0, 0],  "Configuracion normal"),
        ([0, -120, 0, 0, 0, 0],  "J2 fuera de limite"),
        ([0, -100, 50, 0, 0, 0], "Z peligrosa"),
    ]

    print("\nVerificacion de configuraciones de prueba:")
    print(f"{'Descripcion':<30} {'Limites':>10} {'Altura':>10} {'Estado':>10}")
    print("-"*65)
    for angles, desc in tests:
        ok_l, msg_l = checker.check_joint_limits(angles)
        ok_z, z, msg_z = checker.check_min_height(angles)
        estado = "SEGURA" if (ok_l and ok_z) else "PELIGROSA"
        print(f"{desc:<30} {'OK' if ok_l else 'FALLA':>10} {f'Z={z:.0f}mm':>10} {estado:>10}")
