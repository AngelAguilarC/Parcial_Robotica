#!/usr/bin/env python3
# coding: utf-8
"""
P3 - Cinematica Inversa (IK)
MyCobot 280 - 6 DOF

Dos enfoques:
  1. IK numerica via API: mc.send_coords([x,y,z,rx,ry,rz], 30, 1) -> mc.get_angles()
  2. IK analitica simplificada (primeros 3 joints, modelo planar 2R + base)

Verifica ambas soluciones para 3 posiciones cartesianas y tabula el error.
Identifica configuraciones singulares y posiciones fuera del workspace.
"""
import time
import sys
import os
import numpy as np
from math import atan2, sqrt, acos, asin, degrees, radians, pi

# Constantes del robot (de la tabla DH)
L2 = 110.4   # mm - longitud del eslabon 2 (hombro)
L3 = 96.0    # mm - longitud del eslabon 3 (codo)
D1 = 131.56  # mm - altura base (d1)
D4 = 66.39   # mm - offset muneca 1
D5 = 73.18   # mm - offset muneca 2
D6 = 48.60   # mm - longitud gripper

# Alcance maximo teorico del robot
MAX_REACH = L2 + L3 + D4 + D5 + D6   # aprox 395 mm


class InverseKinematics:
    """
    Cinematica inversa para el MyCobot 280.
    Implementa IK analitica simplificada para los 3 primeros joints.
    """

    def __init__(self):
        self.L2 = L2
        self.L3 = L3
        self.d1 = D1

    def is_reachable(self, x, y, z):
        """
        Verifica si la posicion (x,y,z) esta dentro del espacio de trabajo.

        Retorna (bool, str): (alcanzable, mensaje)
        """
        r_xy = sqrt(x**2 + y**2)
        z_prime = z - self.d1
        dist = sqrt(r_xy**2 + z_prime**2)

        if dist > (self.L2 + self.L3):
            return False, f"Fuera de alcance: dist={dist:.1f} mm > max={self.L2+self.L3:.1f} mm"
        if dist < abs(self.L2 - self.L3):
            return False, f"Muy cerca: dist={dist:.1f} mm < min={abs(self.L2-self.L3):.1f} mm"
        if z < 10:
            return False, f"Z={z:.1f} mm por debajo de la mesa (Z_min~10 mm)"
        return True, "Alcanzable"

    def is_singular(self, x, y, z):
        """
        Detecta configuraciones singulares.
        Singular cuando el brazo esta completamente extendido o completamente plegado.
        """
        r_xy = sqrt(x**2 + y**2)
        z_prime = z - self.d1
        dist = sqrt(r_xy**2 + z_prime**2)

        # Singularidad de extension: brazo completamente extendido
        if abs(dist - (self.L2 + self.L3)) < 5.0:
            return True, "Singularidad de extension (brazo completamente estirado)"
        # Singularidad de retraccion: codo alineado con hombro
        if abs(dist - abs(self.L2 - self.L3)) < 5.0:
            return True, "Singularidad de retraccion (codo plegado al maximo)"
        # Singularidad en la base: x=0 y=0
        if r_xy < 1.0:
            return True, "Singularidad en eje Z (x~0, y~0)"
        return False, "No singular"

    def solve_analytical(self, x, y, z, elbow_up=True):
        """
        IK analitica simplificada para los primeros 3 joints.
        Modelo: base rotacional (J1) + brazo planar 2R (J2, J3).
        J4, J5, J6 se fijan a 0 (orientacion neutra).

        Parametros
        ----------
        x, y, z    : float  posicion objetivo en mm
        elbow_up   : bool   True = codo arriba, False = codo abajo

        Retorna
        -------
        list[float]  [theta1..theta6] en grados, o None si no es alcanzable
        """
        reachable, msg = self.is_reachable(x, y, z)
        if not reachable:
            print(f"  [IK Analitica] FALLO: {msg}")
            return None

        # theta1: rotacion de la base alrededor de Z
        theta1 = degrees(atan2(y, x))

        # proyeccion en el plano del brazo
        r = sqrt(x**2 + y**2)
        z_prime = z - self.d1

        # coseno del angulo del codo (ley de cosenos)
        cos_theta3 = (r**2 + z_prime**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_theta3 = max(-1.0, min(1.0, cos_theta3))   # clamp para estabilidad numerica

        # theta3 con signo segun elbow_up / elbow_down
        sin_theta3 = sqrt(1 - cos_theta3**2) if elbow_up else -sqrt(1 - cos_theta3**2)
        theta3 = degrees(atan2(sin_theta3, cos_theta3))

        # theta2
        theta2 = degrees(
            atan2(z_prime, r) - atan2(self.L3 * sin_theta3, self.L2 + self.L3 * cos_theta3)
        )

        # J4, J5, J6 en 0 (orientacion neutra del gripper)
        return [theta1, theta2, theta3, 0.0, 0.0, 0.0]

    def solve_api(self, mc, x, y, z, rx=-175.0, ry=0.0, rz=-45.0, speed=30):
        """
        IK numerica via API del robot.
        Envia las coordenadas al robot y lee los angulos resultantes.

        Parametros
        ----------
        mc    : MyCobot  instancia del robot
        x,y,z : float    posicion objetivo en mm
        rx,ry,rz: float  orientacion en grados (Euler)
        speed : int      velocidad de movimiento (0-100)

        Retorna
        -------
        list[float]  [theta1..theta6] en grados, o None si falla
        """
        coords = [x, y, z, rx, ry, rz]
        mc.send_coords(coords, speed, 1)
        time.sleep(3.0)
        angles = mc.get_angles()
        if angles and len(angles) == 6:
            return list(angles)
        return None


def compare_ik_solutions(mc=None):
    """
    Compara IK analitica vs. solucion API para 3 posiciones cartesianas.
    Tabula el error en grados entre ambas soluciones.
    """
    ik = InverseKinematics()

    # 3 posiciones de prueba en mm (dentro del workspace del MyCobot 280)
    test_positions = [
        (150,   0, 200, "Frente al robot"),
        (100, 100, 180, "Diagonal derecha"),
        (  0, 150, 220, "Lateral izquierdo"),
    ]

    print("\n" + "="*100)
    print("P3 - COMPARACION IK ANALITICA vs. API - MyCobot 280")
    print("="*100)

    results = []
    for x, y, z, desc in test_positions:
        print(f"\n[Posicion: {desc}]  x={x}, y={y}, z={z} mm")

        # Verificar alcanzabilidad y singularidad
        reachable, reach_msg = ik.is_reachable(x, y, z)
        singular, sing_msg = ik.is_singular(x, y, z)
        print(f"  Alcanzable: {reach_msg}")
        print(f"  Singular:   {sing_msg}")

        if not reachable:
            results.append((desc, None, None, None))
            continue

        # IK analitica
        angles_analytical = ik.solve_analytical(x, y, z)
        print(f"  IK Analitica: {[round(a, 2) for a in angles_analytical] if angles_analytical else 'FALLO'}")

        # IK API (requiere hardware)
        angles_api = None
        if mc is not None:
            angles_api = ik.solve_api(mc, x, y, z)
            print(f"  IK API:       {[round(a, 2) for a in angles_api] if angles_api else 'FALLO'}")
        else:
            print(f"  IK API:       N/A (sin hardware)")

        # Error en grados (solo para los 3 primeros joints)
        if angles_analytical and angles_api:
            errors = [abs(angles_analytical[i] - angles_api[i]) for i in range(6)]
            error_total = sqrt(sum(e**2 for e in errors))
            print(f"  Error por joint (deg): {[round(e,3) for e in errors]}")
            print(f"  Error RMS total:       {error_total:.4f} deg")
        else:
            errors = [float('nan')] * 6
            error_total = float('nan')

        results.append((desc, angles_analytical, angles_api, errors))

    print("\n" + "="*100)
    print("TABLA RESUMEN")
    print("="*100)
    print(f"{'Posicion':<22} {'J1 err':>8} {'J2 err':>8} {'J3 err':>8} {'J4 err':>8} {'J5 err':>8} {'J6 err':>8}")
    print("-"*100)
    for desc, ang_a, ang_api, errs in results:
        if errs is None:
            print(f"{desc:<22}  {'FUERA DE WORKSPACE':>52}")
        elif any(np.isnan(errs)):
            print(f"{desc:<22}  {'SIN HARDWARE':>52}")
        else:
            err_str = " ".join(f"{e:>8.3f}" for e in errs)
            print(f"{desc:<22}  {err_str}")
    print("="*100)

    return results


def identify_singular_configs():
    """
    Identifica y explica las configuraciones singulares del MyCobot 280.
    """
    print("\n" + "="*70)
    print("CONFIGURACIONES SINGULARES - MyCobot 280")
    print("="*70)
    singulars = [
        ("Brazo completamente extendido",
         "J1=0, J2=-20, J3=0",
         "El codo esta alineado. Infinitas soluciones de J2/J3 dan la misma pos."),
        ("Singularidad en eje Z (overhead)",
         "J1=cualquier, x~0, y~0",
         "El brazo apunta verticalmente. J1 indefinido (atan2(0,0))."),
        ("Muñeca alinhada (wrist singularity)",
         "J5=0 deg",
         "J4 y J6 se vuelven colineales, perdiendo un DOF de orientacion."),
        ("Retraccion maxima",
         "J3=+150 o J3=-150",
         "Brazo completamente plegado, espacio de movimiento muy reducido."),
    ]
    for name, config, desc in singulars:
        print(f"\n[{name}]")
        print(f"  Ejemplo: {config}")
        print(f"  Efecto:  {desc}")
    print("="*70)


if __name__ == "__main__":
    compare_ik_solutions(mc=None)
    identify_singular_configs()

    # Para uso con hardware:
    # from pymycobot.mycobot import MyCobot
    # mc = MyCobot('/dev/ttyUSB0', 1000000)
    # mc.power_on()
    # time.sleep(0.5)
    # compare_ik_solutions(mc=mc)
