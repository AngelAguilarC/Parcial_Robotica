#!/usr/bin/env python3
# coding: utf-8
"""
P2 - Cinematica Directa (FK)
MyCobot 280 - 6 DOF

Implementa T0_6 = T1*T2*T3*T4*T5*T6 con numpy.
Verifica contra mc.get_coords() y tabula el error en mm.
Grafica el espacio de trabajo alcanzable (nube 2D en plano XZ).
"""
import numpy as np
import time
from math import radians, degrees, cos, sin, sqrt
import sys
import os

# Reutilizamos la logica DH del P1
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'P1_DH'))
from p1_dh_representacion import DH_TABLE, dh_matrix, forward_kinematics, extract_position


class ForwardKinematics:
    """
    Cinematica directa del MyCobot 280.
    Calcula la posicion y orientacion del extremo dado un vector de angulos.
    """

    def __init__(self):
        self.dh_table = DH_TABLE

    def compute(self, thetas_deg):
        """
        Calcula T0_6 para un vector de 6 angulos articulares.

        Parametros
        ----------
        thetas_deg : list[float]  angulos [theta1..theta6] en grados

        Retorna
        -------
        T         : np.ndarray (4x4)
        position  : tuple (x, y, z) en mm
        """
        T = forward_kinematics(thetas_deg)
        position = extract_position(T)
        return T, position

    def workspace_cloud(self, n_samples=5000, seed=42):
        """
        Genera nube de puntos del espacio de trabajo alcanzable (plano XZ).
        Muestrea configuraciones aleatorias dentro de los rangos de cada joint.

        Retorna
        -------
        points_xz : np.ndarray (N, 2) columnas [x, z] en mm
        """
        rng = np.random.default_rng(seed)
        joint_ranges = [
            (-168, 168), (-135, 90), (-150, 150),
            (-145, 145), (-165, 165), (-180, 180),
        ]
        points = []
        for _ in range(n_samples):
            thetas = [rng.uniform(lo, hi) for lo, hi in joint_ranges]
            _, pos = self.compute(thetas)
            points.append((pos[0], pos[2]))   # (x, z)
        return np.array(points)


def run_fk_verification(mc=None):
    """
    Calcula FK para 5 configuraciones de joints, compara con mc.get_coords()
    y tabula el error en mm.

    Parametros
    ----------
    mc : MyCobot instance o None (si None, solo calcula sin hardware)
    """
    fk = ForwardKinematics()

    # 5 configuraciones de prueba (en grados)
    test_configs = [
        [  0,    0,    0,    0,    0,   0],
        [ 45,  -30,   45,  -15,   30,  0],
        [-45,   20,  -60,   30,  -20,  0],
        [ 90,  -45,   90,  -45,   45,  0],
        [ 30,   10,  -30,   10,  -10,  0],
    ]

    print("\n" + "="*90)
    print("P2 - VERIFICACION CINEMATICA DIRECTA")
    print("="*90)
    header = (f"{'Config':<8} {'theta1':>7} {'theta2':>7} {'theta3':>7} "
              f"{'theta4':>7} {'theta5':>7} {'theta6':>7}  |  "
              f"{'x_calc':>8} {'y_calc':>8} {'z_calc':>8}  |  "
              f"{'x_real':>8} {'y_real':>8} {'z_real':>8}  |  {'Error':>7}")
    print(header)
    print("-"*90)

    results = []
    for idx, cfg in enumerate(test_configs):
        T, pos_calc = fk.compute(cfg)
        x_c, y_c, z_c = pos_calc

        if mc is not None:
            mc.send_angles(cfg, 30)
            time.sleep(2.5)
            real = mc.get_coords()
            if real and len(real) >= 3:
                x_r, y_r, z_r = real[0], real[1], real[2]
            else:
                x_r, y_r, z_r = float('nan'), float('nan'), float('nan')
        else:
            # Sin hardware: marcamos como N/A
            x_r, y_r, z_r = float('nan'), float('nan'), float('nan')

        if not any(np.isnan([x_r, y_r, z_r])):
            error = sqrt((x_c - x_r)**2 + (y_c - y_r)**2 + (z_c - z_r)**2)
        else:
            error = float('nan')

        results.append((cfg, (x_c, y_c, z_c), (x_r, y_r, z_r), error))

        th_str = " ".join(f"{t:>7.1f}" for t in cfg)
        if not np.isnan(error):
            row = (f"#{idx+1:<6}  {th_str}    "
                   f"{x_c:>8.2f} {y_c:>8.2f} {z_c:>8.2f}    "
                   f"{x_r:>8.2f} {y_r:>8.2f} {z_r:>8.2f}    "
                   f"{error:>7.3f}")
        else:
            row = (f"#{idx+1:<6}  {th_str}    "
                   f"{x_c:>8.2f} {y_c:>8.2f} {z_c:>8.2f}    "
                   f"{'N/A':>8} {'N/A':>8} {'N/A':>8}    {'N/A':>7}")
        print(row)

    print("="*90)
    print("Nota: Error = distancia euclidiana ||pos_calc - pos_real|| en mm")
    if mc is None:
        print("MODO OFFLINE: conectar hardware para ver error real.")
    return results


def plot_workspace(points_xz):
    """Genera grafica 2D de la nube de puntos en el plano XZ."""
    try:
        import matplotlib.pyplot as plt
        x = points_xz[:, 0]
        z = points_xz[:, 1]
        plt.figure(figsize=(8, 8))
        plt.scatter(x, z, s=1, alpha=0.3, color='steelblue', label='Espacio de trabajo')
        plt.xlabel('X (mm)')
        plt.ylabel('Z (mm)')
        plt.title('Espacio de Trabajo Alcanzable - MyCobot 280\n(Plano XZ, proyeccion 2D)')
        plt.axhline(0, color='k', linewidth=0.5)
        plt.axvline(0, color='k', linewidth=0.5)
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        out_path = os.path.join(os.path.dirname(__file__), 'workspace_xz.png')
        plt.savefig(out_path, dpi=150, bbox_inches='tight')
        plt.show()
        print(f"\nGrafica guardada en: {out_path}")
    except ImportError:
        print("matplotlib no disponible. Instalar con: pip install matplotlib")


if __name__ == "__main__":
    # -----------------------------------------------------------------------
    # Ejecucion standalone (sin hardware conectado)
    # -----------------------------------------------------------------------
    fk = ForwardKinematics()

    # Verificacion offline
    run_fk_verification(mc=None)

    # Generar y graficar espacio de trabajo
    print("\nGenerando espacio de trabajo (5000 muestras aleatorias)...")
    cloud = fk.workspace_cloud(n_samples=5000)
    print(f"Rango X: [{cloud[:,0].min():.1f}, {cloud[:,0].max():.1f}] mm")
    print(f"Rango Z: [{cloud[:,1].min():.1f}, {cloud[:,1].max():.1f}] mm")
    plot_workspace(cloud)

    # -----------------------------------------------------------------------
    # Para verificacion con hardware real, usar:
    # -----------------------------------------------------------------------
    # from pymycobot.mycobot import MyCobot
    # mc = MyCobot('/dev/ttyUSB0', 1000000)
    # mc.power_on()
    # time.sleep(0.5)
    # run_fk_verification(mc=mc)
