
"""
P1 - Representacion Cinematica (DH)

"""

import numpy as np
from math import radians, degrees, cos, sin, atan2, sqrt

# -----------------------------------------------------------------------
# Tabla DH: (a_mm, d_mm, alpha_deg, theta_offset_deg)
# theta_offset se suma al angulo articular antes de computar la matriz.
# Valores verificados con hardware: error euclidiano ~6.4 mm en HOME.
# -----------------------------------------------------------------------
DH_TABLE = [
    (   0,   134.75,  90.0,   0.0),   # J1 Base
    (-110,     0.0,    0.0, -90.0),   # J2 Hombro
    ( -96,     0.0,    0.0,   0.0),   # J3 Codo
    (   0,    63.4,   90.0, -90.0),   # J4 Muneca 1
    (   0,   75.05,  -90.0,  90.0),   # J5 Muneca 2
    (   0,    50.0,    0.0,   0.0),   # J6 Gripper
]

JOINT_RANGES = [
    (-168, 168),
    (-135,  90),
    (-150, 150),
    (-145, 145),
    (-165, 165),
    (-180, 180),
]


def dh_matrix(theta_deg, d, a, alpha_deg):
    """
    Matriz de transformacion homogenea DH para un eslabon.

    """
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


def build_all_Ti(thetas_deg):
    """
    Calcula las 6 matrices individuales Ti (incluye theta_offset de la tabla).
    """
    matrices = []
    for i, (a, d, alpha, offset) in enumerate(DH_TABLE):
        Ti = dh_matrix(thetas_deg[i] + offset, d, a, alpha)
        matrices.append(Ti)
    return matrices


def forward_kinematics(thetas_deg):

    matrices = build_all_Ti(thetas_deg)
    T = np.eye(4)
    for Ti in matrices:
        T = T @ Ti
    return T


def extract_position(T):
    """Extrae la posicion (x, y, z) en mm de la ultima columna de T."""
    return T[0, 3], T[1, 3], T[2, 3]


def print_dh_table():

    print("\n" + "="*72)
    print("TABLA DE PARAMETROS DH - MyCobot 280 (6-DOF)")
    print("="*72)
    header = (f"{'Joint':<12} {'a_i(mm)':>9} {'d_i(mm)':>9} "
              f"{'alpha_i':>10} {'theta_i':>14} {'Rango':>20}")
    print(header)
    print("-"*72)
    joints = ['J1 Base', 'J2 Hombro', 'J3 Codo', 'J4 Muneca1', 'J5 Muneca2', 'J6 Gripper']
    for i, (name, (a, d, alpha, offset)) in enumerate(zip(joints, DH_TABLE)):
        rmin, rmax = JOINT_RANGES[i]
        offset_str = f"q{i+1}{offset:+.0f} deg" if offset != 0.0 else f"q{i+1}"
        row = (f"{name:<12} {a:>9.2f} {d:>9.2f} "
               f"{alpha:>8.1f} deg  {offset_str:>14}  [{rmin:>4}, {rmax:>4}] deg")
        print(row)
    print("="*72)
    print("\nSignificado fisico:")
    significados = [
        "d1=134.75mm: altura de J1 sobre la mesa. alpha1=90 deg: J2 perpendicular a J1.",
        "a2=-110mm: longitud del eslabon hombro. alpha2=0: J2 y J3 paralelos (movimiento planar).",
        "a3=-96mm: longitud del antebrazo. alpha3=0: eje colineal con J2.",
        "d4=63.4mm: offset axial de la muneca 1. alpha4=90 deg: gira el plano de orientacion.",
        "d5=75.05mm: offset axial de la muneca 2. alpha5=-90 deg: invierte el plano.",
        "d6=50mm: longitud TCP-gripper. alpha6=0: eje colineal (roll del gripper).",
    ]
    for i, (name, sig) in enumerate(zip(joints, significados)):
        print(f"  [{name}] {sig}")
    print("="*72)


def verify_home_pose():
    """
    Calcula T0_6 en HOME [0,0,0,0,0,0] y muestra la posicion del extremo.
    Retorna la matriz T para comparacion con hardware.
    """
    home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    T = forward_kinematics(home)
    x, y, z = extract_position(T)
    alcance = sqrt(x**2 + y**2 + z**2)
    print(f"\nVerificacion pose HOME [0, 0, 0, 0, 0, 0]:")
    print(f"  Posicion extremo: x={x:.3f} mm  y={y:.3f} mm  z={z:.3f} mm")
    print(f"  Alcance: ||pos|| = {alcance:.3f} mm")
    print(f"\n  Matriz T0_6:\n{np.round(T, 4)}")
    return T


if __name__ == "__main__":
    print_dh_table()
    verify_home_pose()

    print("\nMatrices Ti individuales en pose HOME:")
    matrices = build_all_Ti([0] * 6)
    for i, Ti in enumerate(matrices):
        print(f"\nT{i+1}:\n{np.round(Ti, 4)}")
