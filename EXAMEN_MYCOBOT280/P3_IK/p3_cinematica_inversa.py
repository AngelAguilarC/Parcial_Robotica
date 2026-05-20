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

import sys
import os
import math
import time
import numpy as np
from typing import Optional, List, Tuple

# Importar modulos de otras practicas
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'P1_DH'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'P2_FK'))

try:
    from p1_dh_representacion import DH_TABLE, forward_kinematics, extract_position
    from p2_cinematica_directa import ForwardKinematics
except ImportError as e:
    print(f" No se pudo importar modulos de cinemática directa: {e}")
    ForwardKinematics = None

# -----------------------------------------------------------------------
# Constantes del MyCobot 280
# -----------------------------------------------------------------------
ALTURA_BASE = 131.56           # d1 en mm
LONGITUD_ESLABÓN_2 = 110.4     # a2 en mm (hombro)
LONGITUD_ESLABÓN_3 = 96.0      # a3 en mm (codo)
D_MUÑECA_Z4 = 66.39            # d4 en mm
D_MUÑECA_Z5 = 73.18            # d5 en mm
LONGITUD_GRIPPER = 48.60       # d6 en mm

# Parametros avanzados (offset de muñeca en plano XZ)
_D_MUÑECA_Z3 = 2.0             # mm - offset mínimo del eje Z
_L3_EFF = math.sqrt(LONGITUD_ESLABÓN_3**2 + D_MUÑECA_Z5**2)  # eslabón efectivo
_ALPHA_MUÑECA = math.atan2(D_MUÑECA_Z5, LONGITUD_ESLABÓN_3)  # ángulo offset

# Tabla DH para optimización numérica 6DOF
PARAMETROS_DH = [
    (0,      131.56,  90.0,   (-168, 168)),    # J1
    (110.4,    0,      0.0,   (-135,  90)),    # J2
    (96,       0,      0.0,   (-150, 150)),    # J3
    (0,     66.39,   -90.0,   (-145, 145)),    # J4
    (0,     73.18,    90.0,   (-165, 165)),    # J5
    (0,     48.60,     0.0,   (-180, 180)),    # J6
]

import logging
registro = logging.getLogger(__name__)


class InverseKinematics:
    """
    Cinematica inversa para el MyCobot 280 con multiples enfoques.
    
    Metodos:
      - resolver():        IK analitica avanzada con offset de muñeca
      - resolver_pdf():    IK analitica simplificada (formula del examen)
      - resolver_via_api(): IK numerica via API del robot
      - resolver_6dof():   IK numerica 6DOF con scipy.optimize
      - comparar_ik():     compara soluciones analitica vs API
    """

    def __init__(self):
        """Inicializa el solucionador IK con cinemática directa."""
        self.cd = ForwardKinematics() if ForwardKinematics else None

    # ====================================================================
    # IK Analitica Avanzada (con offset de muñeca)
    # ====================================================================
    def resolver(self, x: float, y: float, z: float, codo_arriba: bool = True) -> list:
        """
        IK analitica avanzada: considera offset de muñeca (d5).
        Resuelve J1-J3; J4, J5 = 0, J6 = -45.

        Parametros
        ----------
        x, y, z : float
            Posicion objetivo en mm
        codo_arriba : bool
            True = codo arriba, False = codo abajo

        Retorna
        -------
        list : [j1, j2, j3, 0, 0, -45] en grados

        Lanza ValueError si está fuera del workspace.
        """
        rho_xy = math.sqrt(x**2 + y**2)
        if rho_xy < _D_MUÑECA_Z3:
            raise ValueError(
                f"Posición ({x:.1f}, {y:.1f}, {z:.1f}) muy cerca del eje Z — "
                "inaccesible con la muñeca fija."
            )

        # J1: sin(j1+φ) = K/rho donde φ = atan2(-y, x)
        j1 = math.asin(_D_MUÑECA_Z3 / rho_xy) - math.atan2(-y, x)
        c1, s1 = math.cos(j1), math.sin(j1)
        alcance = c1 * x + s1 * y          # proyección sobre dirección del brazo
        
        if alcance < 0:                     # solución alternativa
            j1 = math.pi - math.asin(_D_MUÑECA_Z3 / rho_xy) - math.atan2(-y, x)
            c1, s1 = math.cos(j1), math.sin(j1)
            alcance = c1 * x + s1 * y

        z_prima = z - ALTURA_BASE

        # Problema 2R con L3_eff = sqrt(L3^2 + d5^2)
        r2 = alcance**2 + z_prima**2
        coseno_j3_eff = (r2 - LONGITUD_ESLABÓN_2**2 - _L3_EFF**2) / (
            2 * LONGITUD_ESLABÓN_2 * _L3_EFF
        )
        
        if abs(coseno_j3_eff) > 1.0:
            raise ValueError(
                f"Posición ({x:.1f}, {y:.1f}, {z:.1f}) fuera del espacio de trabajo."
            )

        signo_codo = 1.0 if codo_arriba else -1.0
        seno_j3_eff = signo_codo * math.sqrt(1.0 - coseno_j3_eff**2)
        j3_eff = math.atan2(seno_j3_eff, coseno_j3_eff)

        j2 = (math.atan2(z_prima, alcance)
              - math.atan2(_L3_EFF * seno_j3_eff,
                           LONGITUD_ESLABÓN_2 + _L3_EFF * coseno_j3_eff))
        j3 = j3_eff - _ALPHA_MUÑECA

        return [math.degrees(j1), math.degrees(j2), math.degrees(j3), 0.0, 0.0, -45.0]

    # ====================================================================
    # IK Analitica Simplificada (formula del examen)
    # ====================================================================
    def resolver_pdf(self, x: float, y: float, z: float, codo_arriba: bool = True) -> list:
        """
        IK analitica simplificada — formula exacta del examen (Sec. 3.3).
        θ1 = atan2(y, x), modelo planar 2R para J2-J3.
        J4, J5 = 0, J6 = -45.

        Parametros
        ----------
        x, y, z : float
            Posicion objetivo en mm
        codo_arriba : bool
            True = codo arriba, False = codo abajo

        Retorna
        -------
        list : [j1, j2, j3, 0, 0, -45] en grados

        Lanza ValueError si está fuera del workspace.
        """
        j1 = math.atan2(y, x)

        r = math.sqrt(x**2 + y**2)
        z_prima = z - ALTURA_BASE

        r2 = r**2 + z_prima**2
        cos_j3 = (r2 - LONGITUD_ESLABÓN_2**2 - LONGITUD_ESLABÓN_3**2) / (
            2 * LONGITUD_ESLABÓN_2 * LONGITUD_ESLABÓN_3
        )

        if abs(cos_j3) > 1.0:
            raise ValueError(
                f"Posición ({x:.1f}, {y:.1f}, {z:.1f}) fuera del espacio de trabajo."
            )

        signo = 1.0 if codo_arriba else -1.0
        sin_j3 = signo * math.sqrt(1.0 - cos_j3**2)
        j3 = math.atan2(sin_j3, cos_j3)

        j2 = (math.atan2(z_prima, r)
              - math.atan2(LONGITUD_ESLABÓN_3 * sin_j3,
                           LONGITUD_ESLABÓN_2 + LONGITUD_ESLABÓN_3 * cos_j3))

        return [math.degrees(j1), math.degrees(j2), math.degrees(j3), 0.0, 0.0, -45.0]

    # ====================================================================
    # IK Numerica via API del robot
    # ====================================================================
    def resolver_via_api(self, robot, x: float, y: float, z: float,
                         rx: float = -175.0, ry: float = 0.0, rz: float = -45.0,
                         velocidad: int = 30) -> list:
        """
        IK numerica via API del robot (requiere hardware).

        Parametros
        ----------
        robot : MyCobot
            Instancia conectada del robot
        x, y, z : float
            Posicion objetivo en mm
        rx, ry, rz : float
            Orientacion en grados (Euler XYZ)
        velocidad : int
            Velocidad de movimiento (0-100)

        Retorna
        -------
        list : [j1, j2, j3, j4, j5, j6] en grados
        """
        robot.send_coords([x, y, z, rx, ry, rz], velocidad, 1)
        time.sleep(2.0)
        angles = robot.get_angles()
        return list(angles) if angles else None

    # ====================================================================
    # IK Numerica 6DOF (opcional, requiere scipy)
    # ====================================================================
    def resolver_6dof(self,
                      x: float, y: float, z: float,
                      rx: float = None, ry: float = None, rz: float = None,
                      semilla: list = None,
                      peso_rotacion: float = 50.0,
                      solo_posicion: bool = False,
                      max_iter: int = 500,
                      n_intentos: int = 6) -> list:
        """
        IK numerica con los 6 grados de libertad (requiere scipy).

        Modos:
          - solo_posicion=True: minimiza solo error de posicion
          - rx,ry,rz especificados: minimiza posicion + orientacion Euler XYZ

        Parametros
        ----------
        x, y, z : float
            Posicion objetivo en mm
        rx, ry, rz : float, optional
            Orientacion Euler XYZ en grados
        semilla : list, optional
            Configuración inicial para la optimización
        solo_posicion : bool
            Si True, ignora la orientación
        max_iter : int
            Máximo de iteraciones del optimizador
        n_intentos : int
            Número de semillas distintas a probar

        Retorna
        -------
        list : [j1, j2, j3, j4, j5, j6] en grados

        Lanza RuntimeError si scipy no está disponible o si no converge.
        """
        try:
            from scipy.optimize import least_squares
            from scipy.spatial.transform import Rotation
        except ImportError as e:
            raise RuntimeError(
                "scipy es requerido para IK 6DOF. Instálalo con `pip install scipy`."
            ) from e

        usar_orientacion = (not solo_posicion) and (
            rx is not None or ry is not None or rz is not None
        )
        if usar_orientacion:
            rx = 0.0 if rx is None else rx
            ry = 0.0 if ry is None else ry
            rz = 0.0 if rz is None else rz
            R_objetivo = Rotation.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
        else:
            R_objetivo = None

        pos_objetivo = np.array([x, y, z], dtype=float)
        limites_min = np.array([math.radians(r[0]) for *_, r in PARAMETROS_DH])
        limites_max = np.array([math.radians(r[1]) for *_, r in PARAMETROS_DH])

        def residuo(joints_rad):
            if self.cd is None:
                return np.zeros(3)
            T = self.cd.compute(np.degrees(joints_rad).tolist())[0]
            err_pos = T[:3, 3] - pos_objetivo
            if R_objetivo is None:
                return err_pos
            R_err = T[:3, :3] @ R_objetivo.T
            err_rot = Rotation.from_matrix(R_err).as_rotvec() * peso_rotacion
            return np.concatenate([err_pos, err_rot])

        semillas = []
        if semilla is not None:
            semillas.append(list(semilla))
        for codo in (True, False):
            try:
                semillas.append(self.resolver_pdf(x, y, z, codo_arriba=codo))
            except ValueError:
                pass
        
        rng = np.random.default_rng(42)
        while len(semillas) < n_intentos:
            base = semillas[0] if semillas else [0.0, -30.0, 30.0, 0.0, 0.0, -45.0]
            perturbacion = rng.uniform(-20.0, 20.0, size=6)
            semillas.append([b + p for b, p in zip(base, perturbacion)])

        mejor_joints = None
        mejor_costo = np.inf

        for sem in semillas[:n_intentos]:
            sem_rad = np.clip(np.radians(sem), limites_min, limites_max)
            try:
                res = least_squares(
                    residuo, sem_rad,
                    bounds=(limites_min, limites_max),
                    method='trf', max_nfev=max_iter,
                )
            except Exception:
                continue
            if res.cost < mejor_costo:
                mejor_costo = res.cost
                mejor_joints = np.degrees(res.x).tolist()

        if mejor_joints is None:
            raise ValueError(
                f"IK 6DOF no convergió para ({x:.1f}, {y:.1f}, {z:.1f})."
            )

        if self.cd:
            T_final = self.cd.compute(mejor_joints)[0]
            err_pos_mm = float(np.linalg.norm(T_final[:3, 3] - pos_objetivo))
            if usar_orientacion:
                err_rot_deg = float(np.degrees(np.linalg.norm(
                    Rotation.from_matrix(T_final[:3, :3] @ R_objetivo.T).as_rotvec()
                )))
                if err_pos_mm > 2.0 or err_rot_deg > 5.0:
                    registro.warning(
                        "IK 6DOF residual: err_pos=%.2f mm, err_rot=%.2f° "
                        "(orientación pedida puede ser incompatible).",
                        err_pos_mm, err_rot_deg,
                    )
            elif err_pos_mm > 1.0:
                registro.warning(
                    "IK 6DOF (solo posición) residual: err_pos=%.2f mm.",
                    err_pos_mm,
                )

        return mejor_joints

    # ====================================================================
    # Comparación: IK Analítica vs API
    # ====================================================================
    def comparar_ik(self, robot=None, posiciones: list = None) -> list:
        """
        Compara IK analítica simplificada vs solución API para varias posiciones.

        Parametros
        ----------
        robot : MyCobot, optional
            Instancia del robot (None para solo analítica)
        posiciones : list
            Lista de tuplas (x, y, z) en mm

        Retorna
        -------
        list : resultados con estructura {"xyz": (x,y,z), "analitica": [...], "api": [...], "error_grados": [...]}
        """
        if posiciones is None:
            posiciones = [
                (150, 0, 200),
                (100, 100, 180),
                (80, 80, 220),
            ]

        resultados = []

        for x, y, z in posiciones:
            fila = {"xyz": (x, y, z)}

            # IK Analítica
            try:
                fila["analitica"] = self.resolver_pdf(x, y, z)
            except ValueError as e:
                fila["analitica"] = None
                fila["error"] = str(e)

            # IK vía API
            if robot is not None:
                try:
                    fila["api"] = self.resolver_via_api(robot, x, y, z)
                except Exception as e:
                    fila["api"] = None
                    if "error" not in fila:
                        fila["error"] = str(e)

            # Comparar
            if fila.get("analitica") and fila.get("api"):
                errores = [abs(a - b) for a, b in zip(fila["analitica"], fila["api"])]
                fila["error_grados"] = [round(e, 2) for e in errores]

            resultados.append(fila)

        return resultados


# -----------------------------------------------------------------------
# Funciones de Verificación
# -----------------------------------------------------------------------
def compare_ik_solutions(mc=None):
    """
    Compara IK analitica vs. solucion API para 3 posiciones cartesianas.
    Tabula el error en grados entre ambas soluciones.

    Parametros
    ----------
    mc : MyCobot, optional
        Instancia del robot (None para solo analítica)
    """
    ik = InverseKinematics()

    # 3 posiciones de prueba en mm
    test_positions = [
        (150, 0, 200, "Frente al robot"),
        (100, 100, 180, "Diagonal derecha"),
        (80, 80, 220, "Lateral izquierdo"),
    ]

    print("\n" + "="*100)
    print("P3 - COMPARACION IK ANALITICA vs. API - MyCobot 280")
    print("="*100)

    results = []
    for x, y, z, desc in test_positions:
        print(f"\n[Posicion: {desc}]  x={x}, y={y}, z={z} mm")

        # IK analítica
        try:
            angles_analytical = ik.resolver_pdf(x, y, z)
            print(f"  IK Analitica: {[round(a, 2) for a in angles_analytical]}")
        except ValueError as e:
            print(f"  IK Analitica: FALLO - {e}")
            angles_analytical = None

        # IK vía API
        angles_api = None
        if mc is not None:
            try:
                angles_api = ik.resolver_via_api(mc, x, y, z)
                print(f"  IK API:       {[round(a, 2) for a in angles_api]}")
            except Exception as e:
                print(f"  IK API:       FALLO - {e}")
                angles_api = None

        # Calcular error
        if angles_analytical and angles_api:
            errores = [abs(a - b) for a, b in zip(angles_analytical, angles_api)]
            results.append((desc, angles_analytical, angles_api, errores))
            print(f"  Error (grados): {[round(e, 2) for e in errores]}")
        else:
            results.append((desc, angles_analytical, angles_api, None))

    # Tabla resumen
    print("\n" + "="*100)
    print("TABLA RESUMEN")
    print("="*100)
    print(
        f"{'Posicion':<22} {'J1 err':>8} {'J2 err':>8} {'J3 err':>8} "
        f"{'J4 err':>8} {'J5 err':>8} {'J6 err':>8}"
    )
    print("-"*100)
    for desc, ang_a, ang_api, errs in results:
        if errs:
            err_str = " ".join(f"{e:>8.2f}" for e in errs)
            print(f"{desc:<22} {err_str}")
        else:
            print(f"{desc:<22} {'N/A':>8} {'N/A':>8} {'N/A':>8} {'N/A':>8} {'N/A':>8} {'N/A':>8}")
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
         "El codo está alineado. Infinitas soluciones de J2/J3 dan la misma pos."),
        ("Singularidad en eje Z (overhead)",
         "J1=cualquier, x~0, y~0",
         "El brazo apunta verticalmente. J1 indefinido (atan2(0,0))."),
        ("Muñeca alineada (wrist singularity)",
         "J5=0 deg",
         "J4 y J6 se vuelven colineales, perdiendo un DOF de orientacion."),
        ("Retraccion maxima",
         "J3=+150 o J3=-150",
         "Brazo completamente plegado, espacio de movimiento muy reducido."),
    ]
    for name, config, desc in singulars:
        print(f"\n[{name}]")
        print(f"  Config: {config}")
        print(f"  Desc:   {desc}")
    print("="*70)


# -----------------------------------------------------------------------
# Función Principal
# -----------------------------------------------------------------------
if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )

    print("="*70)
    print("P3 - CINEMATICA INVERSA - MyCobot 280")
    print("="*70)

    # Comparación offline (sin hardware)
    compare_ik_solutions(mc=None)

    # Configuraciones singulares
    identify_singular_configs()

    print("\n[INFO] Para uso con hardware real:")
    print("  from pymycobot.mycobot import MyCobot")
    print("  mc = MyCobot('/dev/ttyUSB0', 1000000)")
    print("  mc.power_on()")
    print("  time.sleep(0.5)")
    print("  compare_ik_solutions(mc=mc)")
