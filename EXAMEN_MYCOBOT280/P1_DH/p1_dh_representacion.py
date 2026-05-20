
import numpy as np
from math import radians, degrees, cos, sin, atan2, sqrt

DH_PARAMS = {
    # Joint: (a_i,  d_i,    alpha_i, theta_offset, range_min, range_max)
    'J1': (  0,    131.56,  90.0,   0.0,  -168,  168),
    'J2': (110.4,    0,      0.0,   0.0,  -135,   90),
    'J3': ( 96,      0,      0.0,   0.0,  -150,  150),
    'J4': (  0,    66.39,  -90.0,   0.0,  -145,  145),
    'J5': (  0,    73.18,   90.0,   0.0,  -165,  165),
    'J6': (  0,    48.6,    0.0,   0.0,  -180,  180),
}

DH_TABLE = [
    # a_i   d_i      alpha_i   (mm and degrees)
    (  0,   131.56,   90.0),   # J1 Base
    (110.4,   0,       0.0),   # J2 Shoulder
    ( 96,     0,       0.0),   # J3 Elbow
    (  0,   66.39,   -90.0),   # J4 Wrist 1
    (  0,   73.18,    90.0),   # J5 Wrist 2
    (  0,   48.60,     0.0),   # J6 Gripper
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

    theta = radians(theta_deg)
    alpha = radians(alpha_deg)

    ct, st = cos(theta), sin(theta)
    ca, sa = cos(alpha), sin(alpha)

    T = np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [ 0,       sa,       ca,      d],
        [ 0,        0,        0,      1],
    ], dtype=float)
    return T


def build_all_Ti(thetas_deg):

    matrices = []
    for i, (a, d, alpha) in enumerate(DH_TABLE):
        Ti = dh_matrix(thetas_deg[i], d, a, alpha)
        matrices.append(Ti)
    return matrices


def forward_kinematics(thetas_deg):

    matrices = build_all_Ti(thetas_deg)
    T = np.eye(4)
    for Ti in matrices:
        T = T @ Ti
    return T


def extract_position(T):
    return T[0, 3], T[1, 3], T[2, 3]


def print_dh_table():
    print("\n" + "="*70)
    print("TABLA DE PARAMETROS DH - MyCobot 280 (6-DOF)")
    print("="*70)
    header = f"{'Joint':<12} {'a_i(mm)':<12} {'d_i(mm)':<12} {'alpha_i':<12} {'theta_i':<12} {'Rango':<20}"
    print(header)
    print("-"*70)
    joints = ['J1 Base', 'J2 Hombro', 'J3 Codo', 'J4 Muneca1', 'J5 Muneca2', 'J6 Gripper']
    for i, (name, (a, d, alpha)) in enumerate(zip(joints, DH_TABLE)):
        rmin, rmax = JOINT_RANGES[i]
        row = f"{name:<12} {a:<12.2f} {d:<12.2f} {alpha:>6.1f} deg   theta_{i+1:<6}  [{rmin}, {rmax}] deg"
        print(row)
    print("="*70)


def verify_home_pose():

    home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    T = forward_kinematics(home)
    x, y, z = extract_position(T)
    print("\nVerificacion pose HOME [0,0,0,0,0,0]:")
    print(f"  Posicion extremo: x={x:.3f} mm, y={y:.3f} mm, z={z:.3f} mm")
    print(f"  Alcance teorico: sqrt(x^2+y^2+z^2) = {sqrt(x**2+y**2+z**2):.3f} mm")
    print(f"\n  Matriz T0_6:\n{np.round(T, 4)}")
    return T


def explain_dh_params():
    """Documenta el significado fisico de cada parametro DH en el MyCobot 280."""
    print("\n" + "="*70)
    print("SIGNIFICADO FISICO DE LOS PARAMETROS DH - MyCobot 280")
    print("="*70)
    explanations = [
        ("J1 Base",
         "d1=131.56mm: altura del plano de J1 sobre la mesa\n"
         "  alpha1=90 deg: el eje de J2 es perpendicular al eje de J1\n"
         "  Permite la rotacion horizontal completa del brazo"),
        ("J2 Hombro",
         "a2=110.4mm: longitud del primer eslabon (hombro)\n"
         "  alpha2=0 deg: J2 y J3 son ejes paralelos (movimiento planar)\n"
         "  Controla la elevacion del codo"),
        ("J3 Codo",
         "a3=96mm: longitud del segundo eslabon (antebrazo)\n"
         "  alpha3=0 deg: eje paralelo a J2 y J2\n"
         "  Dobla el codo del robot"),
        ("J4 Muneca 1",
         "d4=66.39mm: offset axial de la muneca 1\n"
         "  alpha4=-90 deg: gira el plano de movimiento para orientacion\n"
         "  Primer DOF de orientacion de la herramienta"),
        ("J5 Muneca 2",
         "d5=73.18mm: offset axial de la muneca 2\n"
         "  alpha5=90 deg: invierte el plano para el siguiente eje\n"
         "  Segundo DOF de orientacion (pitch de la herramienta)"),
        ("J6 Gripper",
         "d6=48.60mm: distancia TCP al ultimo joint (longitud gripper)\n"
         "  alpha6=0 deg: eje colineal con el eje de la herramienta\n"
         "  Controla la rotacion del gripper (roll)"),
    ]
    for name, desc in explanations:
        print(f"\n[{name}]")
        print(f"  {desc}")
    print("="*70)


if __name__ == "__main__":
    print_dh_table()
    verify_home_pose()
    explain_dh_params()

    # Ejemplo: calcular T individuales para home
    print("\nMatrices Ti individuales en pose home:")
    matrices = build_all_Ti([0]*6)
    for i, Ti in enumerate(matrices):
        print(f"\nT{i+1}:\n{np.round(Ti, 4)}")
