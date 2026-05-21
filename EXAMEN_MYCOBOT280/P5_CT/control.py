#!/usr/bin/env python3
# coding: utf-8
"""
P5 - Control de Trayectorias

Implementa el ciclo de agarre pick-and-place:
  init_pose -> pick_upper -> pick_grasp -> [close] -> pick_upper
            -> place_upper -> place_grasp -> [open] -> place_upper -> init_pose

Ejecuta 5 ciclos consecutivos sin intervencion humana y registra la tasa de exito.
"""

import time
from datetime import datetime

# -----------------------------------------------------------------------
# Poses clave capturadas con el robot real (angulos en grados)
# -----------------------------------------------------------------------
POSES = {
    'init_pose':   [1.14,  -0.96,  -1.31,  -2.10,   0.61, -44.29],
    'pick_upper':  [113.1, -17.7,  -72.7,   4.80,   0.50, -30.90],
    'pick_grasp':  [122.2, -37.8,  -83.3,  30.70,  -5.20, -11.30],
    'place_upper': [-87.5, -47.3,   -0.6, -41.30,   0.40, -42.00],
    'place_grasp': [-86.5, -32.9,  -92.5,  40.00,   0.40, -41.50],
}

# Coordenadas cartesianas de referencia (para IK externa)
COORDS = {
    'pick_z_upper':  170.0,   # mm - altura de aproximacion al objeto
    'pick_z_grasp':  100.0,   # mm - altura de agarre
    'place_z_upper': 170.0,   # mm - altura de aproximacion a zona B
}

GRIPPER_OPEN   = 100   # valor para gripper abierto (0-100)
GRIPPER_CLOSED =   0   # valor para gripper cerrado

MOVE_SPEED    = 90     # velocidad de movimiento (0-100)
GRIPPER_SPEED = 70     # velocidad del gripper
WAIT_MOVE     = 1.5    # segundos de espera tras send_angles
WAIT_GRIP     = 1.0    # segundos de espera tras operacion de gripper
MAX_RETRIES   =  3     # intentos antes de declarar fallo de comunicacion


class RobotController:
    """
    Controlador de trayectorias para el MyCobot 280.
    
    """

    def __init__(self, mc=None):
        self.mc = mc
        self.poses = POSES
        self.move_speed   = MOVE_SPEED
        self.gripper_speed = GRIPPER_SPEED
        self.wait_move    = WAIT_MOVE
        self.wait_grip    = WAIT_GRIP

    # ------------------------------------------------------------------
    # Utilidades internas
    # ------------------------------------------------------------------
    def _retry(self, func, *args):
        """Ejecuta func(*args) con hasta MAX_RETRIES intentos."""
        for attempt in range(1, MAX_RETRIES + 1):
            try:
                return func(*args)
            except Exception as e:
                print(f"    [REINTENTO {attempt}/{MAX_RETRIES}] {e}")
                time.sleep(1.0)
        raise RuntimeError(f"Fallo tras {MAX_RETRIES} intentos: {getattr(func, '__name__', str(func))}")

    # ------------------------------------------------------------------
    # Movimientos basicos
    # ------------------------------------------------------------------
    def goto_pose(self, pose_name, speed=None):
        """
        Mueve el robot a una pose por nombre.

        Parametros
        ----------
        pose_name : str   clave en POSES
        speed     : int   velocidad (0-100), usa MOVE_SPEED si None

        Retorna True si OK, False si la pose no existe.
        """
        angles = self.poses.get(pose_name)
        if angles is None:
            print(f"[CTRL] Pose '{pose_name}' no definida en POSES")
            return False
        spd = speed if speed is not None else self.move_speed
        if self.mc is not None:
            self._retry(self.mc.send_angles, angles, spd)
            time.sleep(self.wait_move)
        else:
            print(f"[CTRL-SIM] goto_pose('{pose_name}', speed={spd})")
        return True

    def open_gripper(self):
        """Abre el gripper."""
        if self.mc is not None:
            self._retry(self.mc.set_gripper_state, 0, self.gripper_speed)
        else:
            print("[CTRL-SIM] open_gripper()")
        time.sleep(self.wait_grip)

    def close_gripper(self):
        """Cierra el gripper."""
        if self.mc is not None:
            self._retry(self.mc.set_gripper_state, 1, self.gripper_speed)
        else:
            print("[CTRL-SIM] close_gripper()")
        time.sleep(self.wait_grip)

    # ------------------------------------------------------------------
    # Operaciones de alto nivel
    # ------------------------------------------------------------------
    def pick(self, x_mm=None, y_mm=None, rx=-175.0, ry=0.0, rz=-45.0):
        """
        Secuencia de agarre: acercar -> bajar -> cerrar gripper -> subir.
        Si x_mm/y_mm son None usa las poses pre-calibradas.

        Retorna True si exitoso.
        """
        try:
            self.open_gripper()
            self.goto_pose('pick_upper')
            self.goto_pose('pick_grasp')
            self.close_gripper()
            self.goto_pose('pick_upper')
            return True
        except RuntimeError as e:
            print(f"[CTRL] Error en pick: {e}")
            return False

    def place(self, place_name='place_grasp'):
        """
        Secuencia de deposito: acercar -> bajar -> abrir gripper -> subir -> init.

        Retorna True si exitoso.
        """
        try:
            self.goto_pose('place_upper')
            self.goto_pose('place_grasp')
            self.open_gripper()
            self.goto_pose('place_upper')
            self.goto_pose('init_pose')
            return True
        except RuntimeError as e:
            print(f"[CTRL] Error en place: {e}")
            return False

    def run_ciclo(self, num):
        """
        Ejecuta un ciclo completo de pick-and-place (11 pasos).
        Maneja errores de comunicacion con reintentos.

        Retorna dict con: ciclo, exito, paso_fallo, tiempo (s)
        """
        secuencia = [
            ('pose',  'init_pose'),    #  1
            ('pose',  'pick_upper'),   #  2
            ('pose',  'pick_grasp'),   #  3
            ('grip',  'close'),        #  4
            ('pose',  'pick_upper'),   #  5
            ('pose',  'init_pose'),    #  6
            ('pose',  'place_upper'),  #  7
            ('pose',  'place_grasp'),  #  8
            ('grip',  'open'),         #  9
            ('pose',  'place_upper'),  # 10
            ('pose',  'init_pose'),    # 11
        ]

        t0 = time.time()
        paso_fallo = None

        for i, (tipo, valor) in enumerate(secuencia):
            etiqueta = f"{'pose' if tipo == 'pose' else 'gripper'}:{valor}"
            print(f"    Paso {i+1:>2}/{len(secuencia)}: {etiqueta}")
            try:
                if tipo == 'grip' and valor == 'close':
                    self.close_gripper()
                elif tipo == 'grip' and valor == 'open':
                    self.open_gripper()
                else:
                    angles = self.poses[valor]
                    if self.mc is not None:
                        self._retry(self.mc.send_angles, angles, self.move_speed)
                        time.sleep(self.wait_move)
            except (RuntimeError, KeyError) as e:
                print(f"    ERROR en paso {i+1}: {e}")
                paso_fallo = i + 1
                try:
                    if self.mc is not None:
                        self.mc.send_angles(self.poses['init_pose'], 30)
                except Exception:
                    pass
                break

        return {
            'ciclo':      num,
            'exito':      paso_fallo is None,
            'paso_fallo': paso_fallo,
            'tiempo':     round(time.time() - t0, 2),
        }


if __name__ == "__main__":
    print("P5 - Control de Trayectorias - MyCobot 280")
    print(f"\nPoses calibradas ({len(POSES)}):")
    for nombre, angles in POSES.items():
        print(f"  {nombre:<14}: {[round(a, 2) for a in angles]}")

    ctrl = RobotController(mc=None)
    print("\nSimulacion de 5 ciclos (sin hardware):")
    log = []
    for n in range(1, 6):
        r = ctrl.run_ciclo(n)
        log.append(r)
        estado = "OK" if r['exito'] else f"FALLO (paso {r['paso_fallo']})"
        print(f"  Ciclo {n}: {estado}  ({r['tiempo']}s)")

    n_ok = sum(1 for r in log if r['exito'])
    tasa = n_ok / len(log) * 100
    print(f"\nTasa de exito: {n_ok}/{len(log)}  ({tasa:.0f}%)")
    print("Resultado:", "APROBADO" if tasa >= 80 else "REQUIERE AJUSTE")
