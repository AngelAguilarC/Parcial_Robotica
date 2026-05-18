
import sys
import os
import numpy as np
from math import radians, degrees, cos, sin, atan2, sqrt, acos

# -----------------------------------------------------------------------
# Parametros del robot (de la tabla DH del examen)
# -----------------------------------------------------------------------
# Tabla DH: (a_mm, d_mm, alpha_deg, theta_offset_deg)
# theta_offset refleja la pose de referencia:
#   q2 = theta2 - 90,  q4 = theta4 - 90,  q5 = theta5 + 90
DH_TABLE = [
    (   0,   134.75,  90.0,   0.0),   # J1 Base
    (-110,     0,      0.0, -90.0),   # J2 Shoulder
    ( -96,     0,      0.0,   0.0),   # J3 Elbow
    (   0,    63.4,   90.0, -90.0),   # J4 Wrist 1
    (   0,   75.05,  -90.0,  90.0),   # J5 Wrist 2
    (   0,    50.0,    0.0,   0.0),   # J6 Gripper
]

JOINT_LIMITS = [
    (-168, 168), (-135, 90), (-150, 150),
    (-145, 145), (-165, 165), (-180, 180),
]

CONSERVATIVE_LIMITS = [
    (-160, 160), (-110, 70), (-120, 120),
    (-130, 130), (-150, 150), (-175, 175),
]

L2 = 110.0   # abs(a2)
L3 = 96.0    # abs(a3)
D1 = 134.75  # d1
Z_MIN_SAFE = 60.0


# -----------------------------------------------------------------------
# ForwardKinematics
# -----------------------------------------------------------------------
class ForwardKinematics:

    def dh_matrix(self, theta_deg, d, a, alpha_deg):
        theta = radians(theta_deg)
        alpha = radians(alpha_deg)
        ct, st = cos(theta), sin(theta)
        ca, sa = cos(alpha), sin(alpha)
        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [ 0,       sa,       ca,      d],
            [ 0,        0,        0,      1],
        ], dtype=float)

    def compute(self, thetas_deg):

        T = np.eye(4)
        for i, (a, d, alpha, offset) in enumerate(DH_TABLE):
            T = T @ self.dh_matrix(thetas_deg[i] + offset, d, a, alpha)
        pos = (T[0, 3], T[1, 3], T[2, 3])
        return T, pos

    def workspace_cloud(self, n_samples=5000, seed=42):
        """Nube de puntos del espacio de trabajo (plano XZ)."""
        rng = np.random.default_rng(seed)
        points = []
        for _ in range(n_samples):
            thetas = [rng.uniform(lo, hi) for lo, hi in JOINT_LIMITS]
            _, pos = self.compute(thetas)
            points.append((pos[0], pos[2]))
        return np.array(points)


# -----------------------------------------------------------------------
# InverseKinematics
# -----------------------------------------------------------------------
class InverseKinematics:
    """Cinematica inversa analitica simplificada para el MyCobot 280."""

    def is_reachable(self, x, y, z):
        r = sqrt(x**2 + y**2)
        zp = z - D1
        dist = sqrt(r**2 + zp**2)
        if dist > (L2 + L3):
            return False, f"Fuera de alcance (dist={dist:.1f} mm)"
        if dist < abs(L2 - L3):
            return False, f"Muy cerca (dist={dist:.1f} mm)"
        if z < Z_MIN_SAFE:
            return False, f"Z={z:.1f} mm < Z_min={Z_MIN_SAFE} mm"
        return True, "Alcanzable"

    def solve_analytical(self, x, y, z, elbow_up=True):

        ok, msg = self.is_reachable(x, y, z)
        if not ok:
            return None

        theta1 = degrees(atan2(y, x))
        r = sqrt(x**2 + y**2)
        zp = z - D1

        c3 = (r**2 + zp**2 - L2**2 - L3**2) / (2 * L2 * L3)
        c3 = max(-1.0, min(1.0, c3))
        s3 = sqrt(1 - c3**2) if elbow_up else -sqrt(1 - c3**2)
        theta3 = degrees(atan2(s3, c3))
        theta2 = degrees(atan2(zp, r) - atan2(L3 * s3, L2 + L3 * c3))

        return [theta1, theta2, theta3, 0.0, 0.0, 0.0]

    def solve_api(self, mc, x, y, z, rx=-175.0, ry=0.0, rz=-45.0, speed=30):
        """IK via API del robot (requiere hardware)."""
        import time
        mc.send_coords([x, y, z, rx, ry, rz], speed, 1)
        time.sleep(3.0)
        angles = mc.get_angles()
        return list(angles) if (angles and len(angles) == 6) else None


# -----------------------------------------------------------------------
# CollisionChecker
# -----------------------------------------------------------------------
class CollisionChecker:
    """Verifica colisiones potenciales antes de ejecutar movimientos."""

    def __init__(self, mc=None):
        self.mc = mc
        self.fk = ForwardKinematics()
        self.z_min = Z_MIN_SAFE

    def check_joint_limits(self, angles):
        for i, angle in enumerate(angles):
            lo, hi = CONSERVATIVE_LIMITS[i]
            if not (lo <= angle <= hi):
                return False, f"J{i+1}={angle:.1f} fuera de [{lo},{hi}]"
        return True, "OK"

    def check_min_height(self, angles):
        _, pos = self.fk.compute(angles)
        z = pos[2]
        if z < self.z_min:
            return False, z, f"Z={z:.1f} < {self.z_min}"
        return True, z, f"Z={z:.1f} OK"

    def safe_move(self, target_angles, speed=40):
        import time
        valid_l, msg_l = self.check_joint_limits(target_angles)
        if not valid_l:
            return False

        valid_z, z, msg_z = self.check_min_height(target_angles)
        if not valid_z:
            return False

        if self.mc is not None:
            waypoint = [0.0, 0.0, -90.0, 95.0, 0.0, -45.0]
            self.mc.send_angles(waypoint, speed)
            time.sleep(2.5)
            self.mc.send_angles(target_angles, speed)
            time.sleep(2.5)
        return True


# -----------------------------------------------------------------------
# Funcion publica: ik_solve
# -----------------------------------------------------------------------
def ik_solve(x, y, z, elbow_up=True):

    ik = InverseKinematics()
    return ik.solve_analytical(x, y, z, elbow_up=elbow_up)


if __name__ == "__main__":
    print("Modulo cinematica.py - MyCobot 280")
    print()

    fk = ForwardKinematics()
    T, pos = fk.compute([0, 0, 0, 0, 0, 0])
    print(f"FK home: {[round(p,2) for p in pos]} mm")

    angles = ik_solve(150, 0, 200)
    print(f"IK (150, 0, 200): {[round(a,2) for a in angles] if angles else 'No alcanzable'}")

    checker = CollisionChecker()
    ok, msg = checker.check_joint_limits([0, 0, 0, 0, 0, 0])
    print(f"Limites home: {msg}")
