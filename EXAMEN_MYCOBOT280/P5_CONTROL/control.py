#!/usr/bin/env python3
# coding: utf-8
"""
P5 - Control de Trayectorias
MyCobot 280 - 6 DOF

Implementa el ciclo de agarre:
  init_pose -> watch_pose -> pick_pose -> place_pose

Funciones: goto_pose(), pick(), place(), run_cycle()
Ejecuta 5 ciclos consecutivos y registra tasa de exito.
Gestiona errores de comunicacion (reintentos, timeouts).
"""
import time
import logging
import signal
import sys
import os
from datetime import datetime

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'P4_COLISIONES'))
from p4_evasion_colisiones import CollisionChecker, SAFE_WAYPOINT_ANGLES

logger = logging.getLogger(__name__)

# -----------------------------------------------------------------------
# Poses clave del ciclo (angulos en grados)
# Calibradas para el laboratorio real, ajustar segun setup
# -----------------------------------------------------------------------
POSES = {
    # Pose inicial de reposo segura (todos los joints en 0, J6=-45 para camara)
    "init_pose":  [  0,   0,   0,   0,   0, -45],
    # Pose de observacion (camara apunta a la zona de trabajo)
    "watch_pose": [ 42,   0,   0, -85,  -7,  -3],
    # Pose pre-agarre (sobre el objeto, antes de bajar)
    "pre_pick":   [  0, -20,  30, -10,   0, -45],
    # Zona de deposito - lugar de colocacion final
    "place_pose": [ 75, 215, 115, -175,  0, -45],
}

# Coordenadas cartesianas de las poses [x, y, z, rx, ry, rz] en mm/grados
COORDS = {
    "watch_coords": [180,   0, 200, -175,   0, -45],
    "pick_z_upper": 170,    # mm - altura antes de bajar a agarrar
    "pick_z_grasp": 115,    # mm - altura de agarre
    "place_coords": [ 75, 215, 115, -175,   0, -45],
}

GRIPPER_OPEN   = 100   # valor abierto (0-100)
GRIPPER_CLOSED = 20    # valor cerrado
GRIPPER_SPEED  = 80    # velocidad del gripper
MOVE_SPEED     = 40    # velocidad de movimiento

MAX_RETRIES    = 3
RETRY_DELAY    = 1.0


class RobotController:
    """
    Controlador de trayectorias para el MyCobot 280.
    Implementa el ciclo de agarre con manejo de errores.
    Basado en GraspController del src/jetcobot_utils.
    """

    def __init__(self, mc):
        self.mc = mc
        self.checker = CollisionChecker(mc=mc)
        self.cycle_log = []
        self._setup_emergency_stop()

    def _setup_emergency_stop(self):
        """Registra Ctrl+C como parada de emergencia."""
        def handler(sig, frame):
            logger.warning("[EMERGENCIA] Ctrl+C detectado. Moviendo a pose segura...")
            try:
                self.mc.send_angles(POSES["init_pose"], 30)
                time.sleep(2)
            except Exception:
                pass
            sys.exit(0)
        signal.signal(signal.SIGINT, handler)

    # ------------------------------------------------------------------
    # Utilidades de bajo nivel
    # ------------------------------------------------------------------
    def _send_with_retry(self, func, *args, retries=MAX_RETRIES):
        """Ejecuta func con reintentos ante fallos de comunicacion."""
        for attempt in range(retries):
            try:
                result = func(*args)
                return result
            except Exception as e:
                logger.warning(f"Reintento {attempt+1}/{retries}: {e}")
                time.sleep(RETRY_DELAY)
        raise RuntimeError(f"Fallo tras {retries} intentos")

    def open_gripper(self, delay=1.5):
        self._send_with_retry(self.mc.set_gripper_value, GRIPPER_OPEN, GRIPPER_SPEED)
        if delay > 0:
            time.sleep(delay)

    def close_gripper(self, delay=1.5):
        self._send_with_retry(self.mc.set_gripper_value, GRIPPER_CLOSED, GRIPPER_SPEED)
        if delay > 0:
            time.sleep(delay)

    def wait_movement(self, seconds=2.5):
        time.sleep(seconds)

    # ------------------------------------------------------------------
    # goto_pose: mueve a una pose predefinida con verificacion de colision
    # ------------------------------------------------------------------
    def goto_pose(self, pose_name, speed=MOVE_SPEED):
        """
        Mueve el robot a una pose predefinida.

        Parametros
        ----------
        pose_name : str  nombre de la pose en POSES
        speed     : int  velocidad de movimiento (0-100)

        Retorna
        -------
        bool  True si el movimiento fue ejecutado
        """
        angles = POSES.get(pose_name)
        if angles is None:
            logger.error(f"Pose '{pose_name}' no encontrada")
            return False

        logger.info(f"goto_pose: {pose_name} -> {angles}")
        ok = self.checker.safe_move(angles, speed=speed)
        if ok:
            self.wait_movement(2.5)
        return ok

    def goto_coords(self, coords, speed=MOVE_SPEED):
        """
        Mueve el robot a coordenadas cartesianas con verificacion de colision.
        """
        logger.info(f"goto_coords: {coords}")
        ok = self.checker.safe_coords(coords, speed=speed)
        if ok:
            self.wait_movement(2.5)
        return ok

    # ------------------------------------------------------------------
    # pick: secuencia de agarre
    # ------------------------------------------------------------------
    def pick(self, x, y, rx=-175.0, ry=0.0, rz=-45.0):
        """
        Secuencia de agarre en la posicion (x, y).

        Secuencia:
          1. Abrir gripper
          2. Ir a posicion upper (z=pick_z_upper)
          3. Bajar a posicion de agarre (z=pick_z_grasp)
          4. Cerrar gripper
          5. Subir a upper

        Retorna
        -------
        bool  True si el agarre fue exitoso
        """
        z_upper = COORDS["pick_z_upper"]
        z_grasp = COORDS["pick_z_grasp"]

        try:
            logger.info(f"[PICK] Abriendo gripper")
            self.open_gripper(1.0)

            logger.info(f"[PICK] Moviendo a upper ({x:.1f},{y:.1f},{z_upper})")
            upper_coords = [x, y, z_upper, rx, ry, rz]
            ok = self.goto_coords(upper_coords)
            if not ok:
                return False

            logger.info(f"[PICK] Bajando a posicion de agarre (Z={z_grasp})")
            self._send_with_retry(self.mc.send_coord, 3, z_grasp, MOVE_SPEED)
            self.wait_movement(1.5)

            logger.info("[PICK] Cerrando gripper")
            self.close_gripper(1.5)

            logger.info(f"[PICK] Subiendo a upper (Z={z_upper})")
            self._send_with_retry(self.mc.send_coord, 3, z_upper, MOVE_SPEED)
            self.wait_movement(2.0)

            return True

        except Exception as e:
            logger.error(f"[PICK] Error: {e}")
            return False

    # ------------------------------------------------------------------
    # place: secuencia de deposito
    # ------------------------------------------------------------------
    def place(self, place_name="place_pose"):
        """
        Secuencia de deposito en la zona predefinida.

        Secuencia:
          1. Ir a waypoint de clearance
          2. Moverse a la zona de deposito
          3. Abrir gripper
          4. Subir

        Retorna
        -------
        bool  True si el deposito fue exitoso
        """
        try:
            logger.info(f"[PLACE] Waypoint de clearance")
            self._send_with_retry(self.mc.send_angles, SAFE_WAYPOINT_ANGLES, MOVE_SPEED)
            self.wait_movement(2.5)

            logger.info(f"[PLACE] Moviendose a zona de deposito: {place_name}")
            coords = COORDS["place_coords"]
            self._send_with_retry(self.mc.send_coords, coords, MOVE_SPEED, 1)
            self.wait_movement(3.0)

            logger.info("[PLACE] Abriendo gripper")
            self.open_gripper(1.0)

            logger.info("[PLACE] Subiendo del deposito")
            self._send_with_retry(self.mc.send_coord, 3, COORDS["pick_z_upper"], MOVE_SPEED)
            self.wait_movement(1.5)

            return True

        except Exception as e:
            logger.error(f"[PLACE] Error: {e}")
            return False

    # ------------------------------------------------------------------
    # run_cycle: ciclo completo de pick & place
    # ------------------------------------------------------------------
    def run_cycle(self, pick_x, pick_y, cycle_num=1):
        """
        Ejecuta un ciclo completo: init -> watch -> pick -> place -> init.

        Parametros
        ----------
        pick_x, pick_y : float  coordenadas del objeto a agarrar (mm)
        cycle_num      : int    numero de ciclo (para log)

        Retorna
        -------
        dict  {ciclo, exito, tiempo_total, fases}
        """
        t_start = time.time()
        cycle_result = {
            "ciclo": cycle_num,
            "exito": False,
            "tiempo_total": 0,
            "timestamp": datetime.now().isoformat(),
            "fases": {
                "init":  False,
                "watch": False,
                "pick":  False,
                "place": False,
            }
        }

        logger.info(f"\n{'='*60}")
        logger.info(f"CICLO #{cycle_num} - pick=({pick_x:.1f}, {pick_y:.1f})")
        logger.info(f"{'='*60}")

        # Fase 1: init_pose
        logger.info(f"[CICLO {cycle_num}] Fase 1: init_pose")
        if not self.goto_pose("init_pose"):
            logger.error("Fallo en init_pose")
            cycle_result["tiempo_total"] = time.time() - t_start
            return cycle_result
        cycle_result["fases"]["init"] = True

        # Fase 2: watch_pose
        logger.info(f"[CICLO {cycle_num}] Fase 2: watch_pose")
        if not self.goto_pose("watch_pose"):
            logger.error("Fallo en watch_pose")
            cycle_result["tiempo_total"] = time.time() - t_start
            return cycle_result
        cycle_result["fases"]["watch"] = True

        # Fase 3: pick
        logger.info(f"[CICLO {cycle_num}] Fase 3: pick ({pick_x:.1f}, {pick_y:.1f})")
        if not self.pick(pick_x, pick_y):
            logger.error("Fallo en pick")
            cycle_result["tiempo_total"] = time.time() - t_start
            return cycle_result
        cycle_result["fases"]["pick"] = True

        # Fase 4: place
        logger.info(f"[CICLO {cycle_num}] Fase 4: place")
        if not self.place():
            logger.error("Fallo en place")
            cycle_result["tiempo_total"] = time.time() - t_start
            return cycle_result
        cycle_result["fases"]["place"] = True

        # Volver a init
        self.goto_pose("init_pose")

        cycle_result["exito"] = True
        cycle_result["tiempo_total"] = time.time() - t_start
        logger.info(f"[CICLO {cycle_num}] COMPLETADO en {cycle_result['tiempo_total']:.1f}s")
        return cycle_result


def run_5_cycles(mc, pick_x=150.0, pick_y=0.0):
    """
    Ejecuta 5 ciclos consecutivos sin intervencion humana y registra metricas.

    Parametros
    ----------
    mc           : MyCobot  instancia conectada al robot
    pick_x/pick_y: float    posicion del objeto en mm
    """
    ctrl = RobotController(mc)
    all_results = []
    n_cycles = 5

    # Asegurar inicio seguro
    mc.power_on()
    time.sleep(0.5)
    assert mc.is_controller_connected(), "Error de conexion con el controlador"

    mc.send_angles(POSES["init_pose"], 50)
    time.sleep(3)
    logger.info(f"Posicion inicial: {mc.get_coords()}")

    for i in range(1, n_cycles + 1):
        result = ctrl.run_cycle(pick_x, pick_y, cycle_num=i)
        all_results.append(result)
        ctrl.cycle_log.append(result)

        if not result["exito"]:
            logger.warning(f"Ciclo {i} fallo. Esperando 3s antes del siguiente...")
            mc.send_angles(POSES["init_pose"], 40)
            time.sleep(3)

    # -----------------------------------------------------------------------
    # Reporte de metricas
    # -----------------------------------------------------------------------
    n_ok = sum(1 for r in all_results if r["exito"])
    tasa = n_ok / n_cycles * 100
    tiempos = [r["tiempo_total"] for r in all_results]
    t_avg = sum(tiempos) / len(tiempos)

    print("\n" + "="*70)
    print(f"REPORTE DE 5 CICLOS - MyCobot 280")
    print("="*70)
    print(f"Ciclos exitosos: {n_ok}/{n_cycles}  ({tasa:.0f}%)")
    print(f"Tiempo promedio por ciclo: {t_avg:.1f}s")
    print(f"\n{'Ciclo':<8} {'Exito':<8} {'Tiempo(s)':<12} {'Init':<8} {'Watch':<8} {'Pick':<8} {'Place':<8}")
    print("-"*70)
    for r in all_results:
        f = r["fases"]
        print(f"{r['ciclo']:<8} {'SI' if r['exito'] else 'NO':<8} "
              f"{r['tiempo_total']:<12.1f} "
              f"{'OK' if f['init'] else 'FALLO':<8} "
              f"{'OK' if f['watch'] else 'FALLO':<8} "
              f"{'OK' if f['pick'] else 'FALLO':<8} "
              f"{'OK' if f['place'] else 'FALLO':<8}")
    print("="*70)

    if tasa >= 80:
        print(f"RESULTADO: APROBADO (tasa {tasa:.0f}% >= 80%)")
    else:
        print(f"RESULTADO: REQUIERE AJUSTE (tasa {tasa:.0f}% < 80%)")

    return all_results


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('control_log.txt'),
            logging.StreamHandler()
        ]
    )

    print("P5 - Control de Trayectorias - MyCobot 280")
    print("Para ejecutar con hardware:")
    print("  from pymycobot.mycobot import MyCobot")
    print("  mc = MyCobot('/dev/ttyUSB0', 1000000)")
    print("  run_5_cycles(mc, pick_x=150, pick_y=0)")
