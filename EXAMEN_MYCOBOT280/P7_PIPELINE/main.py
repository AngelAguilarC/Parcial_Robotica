#!/usr/bin/env python3
# coding: utf-8
"""
P7 - Pipeline End-to-End
MyCobot 280 - Lider de Integracion

Maquina de estados:
  IDLE -> DETECTAR -> CALC_IK -> AGARRAR -> DEPOSITAR -> IDLE

Integra:
  - Vision (P6):      detect_object(frame) -> (x_mm, y_mm)
  - Cinematica (P3):  ik_solve(x, y, z)   -> angles
  - Control (P5):     pick(), place(), goto_pose()

Ejecuta al menos 5 ciclos autonomos completos.
"""
import sys
import os
import time
import logging
import signal
import cv2
import numpy as np
from datetime import datetime
from enum import Enum, auto

# Modulos del proyecto
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'P5_CONTROL'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'P6_VISION'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'cinematica'))

from control import RobotController, POSES, MOVE_SPEED
from vision import ObjectDetector

# -----------------------------------------------------------------------
# Configuracion de logging centralizado con timestamp
# -----------------------------------------------------------------------
def setup_logging(log_file="session_log.txt"):
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        handlers=[
            logging.FileHandler(log_file),
            logging.StreamHandler(),
        ]
    )

logger = logging.getLogger("Pipeline")


# -----------------------------------------------------------------------
# Maquina de estados
# -----------------------------------------------------------------------
class State(Enum):
    IDLE       = auto()
    DETECTAR   = auto()
    CALC_IK    = auto()
    AGARRAR    = auto()
    DEPOSITAR  = auto()
    ERROR      = auto()
    DONE       = auto()


class Pipeline:
    """
    Pipeline end-to-end para el MyCobot 280.
    Integra vision, cinematica inversa y control de trayectorias.

    Maquina de estados:
      IDLE -> DETECTAR -> CALC_IK -> AGARRAR -> DEPOSITAR -> IDLE
                |           |           |           |
              ERROR       ERROR       ERROR       ERROR
    """

    def __init__(self, mc, camera_id=0, target_color="red",
                 pick_z=170.0, n_cycles=5):
        self.mc = mc
        self.camera_id = camera_id
        self.target_color = target_color
        self.pick_z = pick_z
        self.n_cycles = n_cycles

        # Modulos
        self.detector = ObjectDetector(target_color=target_color)
        self.controller = RobotController(mc)

        # Estado
        self.state = State.IDLE
        self.current_cycle = 0
        self.cycle_results = []

        # Datos entre estados
        self._detected_pos = None   # (x_mm, y_mm) del objeto
        self._ik_angles    = None   # [theta1..theta6]

        # Camara
        self.cap = None
        self._setup_emergency_stop()

    def _setup_emergency_stop(self):
        def handler(sig, frame):
            logger.warning("[EMERGENCIA] Deteniendo robot...")
            try:
                self.mc.send_angles(POSES["init_pose"], 30)
                time.sleep(2)
            except Exception:
                pass
            if self.cap:
                self.cap.release()
            cv2.destroyAllWindows()
            sys.exit(0)
        signal.signal(signal.SIGINT, handler)

    # ------------------------------------------------------------------
    # Verificacion de conexion al startup
    # ------------------------------------------------------------------
    def startup_check(self):
        """Verifica conexion y mueve a pose inicial."""
        logger.info("[Startup] Verificando conexion con el controlador...")
        if not self.mc.is_controller_connected():
            raise RuntimeError("Error de conexion con el controlador del robot")
        logger.info("[Startup] Conexion OK")

        self.mc.power_on()
        time.sleep(0.5)

        logger.info("[Startup] Moviendo a pose inicial...")
        self.mc.send_angles(POSES["init_pose"], 50)
        time.sleep(3)
        logger.info(f"[Startup] Posicion actual: {self.mc.get_coords()}")

    # ------------------------------------------------------------------
    # Inicializar camara
    # ------------------------------------------------------------------
    def _open_camera(self):
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            raise RuntimeError(f"No se pudo abrir camara {self.camera_id}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        logger.info(f"[Camera] Camara {self.camera_id} abierta")

    def _read_frame(self, retries=3):
        for _ in range(retries):
            ret, frame = self.cap.read()
            if ret and frame is not None:
                return frame
            time.sleep(0.1)
        return None

    # ------------------------------------------------------------------
    # Transiciones de estado
    # ------------------------------------------------------------------
    def _state_idle(self):
        """Estado IDLE: preparar ciclo."""
        logger.info(f"\n{'='*60}")
        logger.info(f"CICLO #{self.current_cycle + 1} - Estado: IDLE")
        logger.info(f"{'='*60}")
        self.controller.goto_pose("watch_pose")
        time.sleep(1.0)
        self._detected_pos = None
        self._ik_angles = None
        self.state = State.DETECTAR

    def _state_detectar(self):
        """Estado DETECTAR: capturar imagen y detectar objeto."""
        logger.info("[DETECTAR] Capturando imagen...")

        MAX_DETECTION_ATTEMPTS = 10
        for attempt in range(MAX_DETECTION_ATTEMPTS):
            frame = self._read_frame()
            if frame is None:
                logger.warning("[DETECTAR] Frame nulo, reintentando...")
                time.sleep(0.5)
                continue

            result = self.detector.detect_object(frame)

            # Mostrar frame de debug
            debug = self.detector.get_debug_frame()
            if debug is not None:
                cv2.imshow("Pipeline Vision", debug)
                cv2.waitKey(1)

            if result is not None:
                x_mm, y_mm = result
                logger.info(f"[DETECTAR] Objeto detectado en ({x_mm:.1f}, {y_mm:.1f}) mm")
                self._detected_pos = (x_mm, y_mm)
                self.state = State.CALC_IK
                return

            logger.debug(f"[DETECTAR] Intento {attempt+1}/{MAX_DETECTION_ATTEMPTS}: no detectado")
            time.sleep(0.3)

        logger.warning("[DETECTAR] Objeto no encontrado tras todos los intentos")
        self.state = State.ERROR

    def _state_calc_ik(self):
        """Estado CALC_IK: calcular cinematica inversa."""
        x_mm, y_mm = self._detected_pos
        logger.info(f"[CALC_IK] Calculando IK para ({x_mm:.1f}, {y_mm:.1f}, {self.pick_z:.1f}) mm")

        try:
            from cinematica import ik_solve
            angles = ik_solve(x_mm, y_mm, self.pick_z)
        except ImportError:
            # Fallback: usar IK analitica del P3
            sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'P3_IK'))
            from p3_cinematica_inversa import InverseKinematics
            ik = InverseKinematics()
            angles = ik.solve_analytical(x_mm, y_mm, self.pick_z)

        if angles is None:
            logger.error(f"[CALC_IK] IK no encontro solucion para ({x_mm:.1f},{y_mm:.1f},{self.pick_z:.1f})")
            self.state = State.ERROR
            return

        logger.info(f"[CALC_IK] Solucion: {[round(a,2) for a in angles]}")
        self._ik_angles = angles
        self.state = State.AGARRAR

    def _state_agarrar(self):
        """Estado AGARRAR: ejecutar secuencia de pick."""
        x_mm, y_mm = self._detected_pos
        logger.info(f"[AGARRAR] Ejecutando pick en ({x_mm:.1f}, {y_mm:.1f}) mm")

        ok = self.controller.pick(x_mm, y_mm)
        if ok:
            logger.info("[AGARRAR] Pick exitoso")
            self.state = State.DEPOSITAR
        else:
            logger.error("[AGARRAR] Pick fallido")
            self.state = State.ERROR

    def _state_depositar(self):
        """Estado DEPOSITAR: ejecutar secuencia de place."""
        logger.info("[DEPOSITAR] Ejecutando place...")
        ok = self.controller.place()
        if ok:
            logger.info("[DEPOSITAR] Place exitoso")
        else:
            logger.error("[DEPOSITAR] Place fallido")
        # Siempre volver a IDLE (ciclo siguiente o fin)
        self.state = State.IDLE
        return ok

    def _state_error(self):
        """Estado ERROR: recuperacion, volver a pose segura."""
        logger.warning("[ERROR] Recuperando a pose segura...")
        try:
            self.mc.send_angles(POSES["init_pose"], 30)
            time.sleep(2.5)
        except Exception as e:
            logger.error(f"[ERROR] No se pudo recuperar: {e}")
        self.state = State.IDLE

    # ------------------------------------------------------------------
    # Loop principal
    # ------------------------------------------------------------------
    def run(self):
        """
        Ejecuta n_cycles ciclos autonomos completos.
        """
        self.startup_check()
        self._open_camera()

        logger.info(f"\nIniciando pipeline: {self.n_cycles} ciclos autonomos")
        logger.info(f"Color objetivo: {self.target_color}")

        t_session_start = time.time()

        while self.current_cycle < self.n_cycles:
            t_cycle_start = time.time()
            cycle_ok = False
            error_state_entered = False

            # Ejecutar la maquina de estados hasta completar un ciclo
            self.state = State.IDLE
            while self.state not in (State.DONE,):
                if self.state == State.IDLE:
                    self._state_idle()

                elif self.state == State.DETECTAR:
                    self._state_detectar()

                elif self.state == State.CALC_IK:
                    self._state_calc_ik()

                elif self.state == State.AGARRAR:
                    self._state_agarrar()

                elif self.state == State.DEPOSITAR:
                    cycle_ok = self._state_depositar()
                    break   # ciclo terminado

                elif self.state == State.ERROR:
                    error_state_entered = True
                    self._state_error()
                    break   # abortar ciclo

            t_cycle = time.time() - t_cycle_start
            self.current_cycle += 1

            result = {
                "ciclo":   self.current_cycle,
                "exito":   cycle_ok and not error_state_entered,
                "tiempo":  t_cycle,
                "timestamp": datetime.now().isoformat(),
            }
            self.cycle_results.append(result)
            status = "OK" if result["exito"] else "FALLO"
            logger.info(f"Ciclo {self.current_cycle}/{self.n_cycles}: {status} "
                        f"({t_cycle:.1f}s)")

        # Pose final de reposo
        self.mc.send_angles(POSES["init_pose"], 40)
        time.sleep(2)
        self.cap.release()
        cv2.destroyAllWindows()

        self._print_session_report(time.time() - t_session_start)
        return self.cycle_results

    # ------------------------------------------------------------------
    # Reporte de sesion
    # ------------------------------------------------------------------
    def _print_session_report(self, total_time):
        n_ok = sum(1 for r in self.cycle_results if r["exito"])
        tasa = n_ok / len(self.cycle_results) * 100 if self.cycle_results else 0

        logger.info("\n" + "="*70)
        logger.info("REPORTE DE SESION - Pipeline MyCobot 280")
        logger.info("="*70)
        logger.info(f"Total de ciclos:   {len(self.cycle_results)}")
        logger.info(f"Ciclos exitosos:   {n_ok} ({tasa:.0f}%)")
        logger.info(f"Tiempo total:      {total_time:.1f}s")
        logger.info(f"Tiempo prom/ciclo: {total_time/max(len(self.cycle_results),1):.1f}s")
        logger.info("")
        logger.info(f"{'Ciclo':<8} {'Estado':<10} {'Tiempo(s)':<12}")
        logger.info("-"*35)
        for r in self.cycle_results:
            logger.info(f"{r['ciclo']:<8} {'OK' if r['exito'] else 'FALLO':<10} {r['tiempo']:<12.1f}")
        logger.info("="*70)
        if tasa >= 80:
            logger.info(f"RESULTADO FINAL: APROBADO ({tasa:.0f}% >= 80%)")
        else:
            logger.info(f"RESULTADO FINAL: REQUIERE AJUSTE ({tasa:.0f}% < 80%)")


# -----------------------------------------------------------------------
# Punto de entrada
# -----------------------------------------------------------------------
if __name__ == "__main__":
    setup_logging("session_log.txt")

    logger.info("P7 - Pipeline End-to-End - MyCobot 280")
    logger.info("Para ejecutar con hardware:")
    logger.info("")
    logger.info("  from pymycobot.mycobot import MyCobot")
    logger.info("  mc = MyCobot('/dev/ttyUSB0', 1000000)")
    logger.info("  pipeline = Pipeline(mc, camera_id=0, target_color='red', n_cycles=5)")
    logger.info("  pipeline.run()")
    logger.info("")
    logger.info("Modulos integrados:")
    logger.info("  - P6 Vision:      detect_object(frame) -> (x_mm, y_mm)")
    logger.info("  - P3 Cinematica:  ik_solve(x, y, z)   -> angles")
    logger.info("  - P5 Control:     pick(), place(), goto_pose()")
    logger.info("")
    logger.info("Maquina de estados:")
    logger.info("  IDLE -> DETECTAR -> CALC_IK -> AGARRAR -> DEPOSITAR -> IDLE")
