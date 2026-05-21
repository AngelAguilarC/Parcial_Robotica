#!/usr/bin/env python3
# coding: utf-8
"""
main.py  -  Pipeline Unificado MyCobot 280

"""

import sys
import os
import signal
import logging
import time
from datetime import datetime
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Tuple

# -----------------------------------------------------------------------
# Rutas de modulos
# -----------------------------------------------------------------------
_BASE = os.path.dirname(os.path.abspath(__file__))

_DIRS = {
    'P1': os.path.join(_BASE, '..', 'P1_DH'),
    'P2': os.path.join(_BASE, '..', 'P2_FK'),
    'P3': os.path.join(_BASE, '..', 'P3_IK'),
    'P4': os.path.join(_BASE, '..', 'P4_COLISIONES'),
    'P5': os.path.join(_BASE, '..', 'P5_CT'),
    'P6': os.path.join(_BASE, '..', 'P6_VISION'),
    'CIN': os.path.join(_BASE, '..', 'cinematica'),
}
for d in _DIRS.values():
    sys.path.insert(0, d)

# -----------------------------------------------------------------------
# Logging centralizado
# -----------------------------------------------------------------------
LOG_FILE = os.path.join(_BASE, 'sesion.log')
logging.basicConfig(
    filename=LOG_FILE,
    level=logging.INFO,
    format='%(asctime)s [%(name)s] %(levelname)s %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
)
logger = logging.getLogger('main')

# -----------------------------------------------------------------------
# Importar modulos P1-P6 (con fallback si no estan disponibles)
# -----------------------------------------------------------------------

def _try_import(name, *symbols):
    """Intenta importar un modulo y retorna (modulo, True) o (None, False)."""
    try:
        mod = __import__(name)
        return mod, True
    except ImportError as e:
        print(f"[WARN] Modulo '{name}' no disponible: {e}")
        logger.warning("Modulo %s no disponible: %s", name, e)
        return None, False


_p1_mod, P1_OK = _try_import('p1_dh_representacion')
_p2_mod, P2_OK = _try_import('p2_cinematica_directa')
_p3_mod, P3_OK = _try_import('p3_cinematica_inversa')
_p4_mod, P4_OK = _try_import('p4_evasion_colisiones')
_p5_mod, P5_OK = _try_import('control')
_p6_mod, P6_OK = _try_import('vision')

print(f"\n[MAIN] Modulos disponibles: "
      f"P1={P1_OK} P2={P2_OK} P3={P3_OK} P4={P4_OK} P5={P5_OK} P6={P6_OK}")
logger.info("Modulos cargados: P1=%s P2=%s P3=%s P4=%s P5=%s P6=%s",
            P1_OK, P2_OK, P3_OK, P4_OK, P5_OK, P6_OK)


# -----------------------------------------------------------------------
# Conexion al robot
# -----------------------------------------------------------------------
def connect_robot():
    """
    Intenta conectar al MyCobot 280. Retorna la instancia o None (simulacion).
    """
    try:
        from pymycobot.mycobot import MyCobot
        mc = MyCobot('/dev/ttyUSB0', 1000000)
        mc.power_on()
        time.sleep(0.5)
        if mc.is_controller_connected():
            print("[MAIN] Robot conectado: MyCobot 280 @ /dev/ttyUSB0")
            logger.info("Robot conectado")
            return mc
        else:
            print("[WARN] Robot no responde al handshake")
    except Exception as e:
        print(f"[WARN] Robot no disponible ({e})")
    print("[MAIN] Modo simulacion activo (sin hardware)")
    logger.warning("Ejecutando en modo simulacion")
    return None


# -----------------------------------------------------------------------
# Manejador de senal Ctrl+C (seguridad)
# -----------------------------------------------------------------------
_mc_global = None


def _emergency_stop(signum, frame):
    print("\n[SEGURIDAD] Ctrl+C detectado - iniciando parada de emergencia")
    logger.warning("Parada de emergencia por Ctrl+C")
    if _mc_global is not None:
        try:
            _mc_global.release_all_servos()
            print("[SEGURIDAD] Servos liberados")
        except Exception:
            pass
    sys.exit(0)


signal.signal(signal.SIGINT, _emergency_stop)


# -----------------------------------------------------------------------
# P1 - Representacion DH
# -----------------------------------------------------------------------
def demo_p1(mc):
    _sep("P1 - REPRESENTACION CINEMATICA (DH)")
    if not P1_OK:
        print("[SKIP] p1_dh_representacion no disponible")
        return

    _p1_mod.print_dh_table()
    T_home = _p1_mod.verify_home_pose()

    if mc is not None:
        mc.send_angles([0, 0, 0, 0, 0, 0], 25)
        time.sleep(3.0)
        coords = mc.get_coords()
        x_calc, y_calc, z_calc = _p1_mod.extract_position(T_home)
        if coords:
            import math
            err = math.sqrt(
                (x_calc - coords[0])**2 +
                (y_calc - coords[1])**2 +
                (z_calc - coords[2])**2
            )
            print(f"\n  Robot get_coords(): {coords[:3]}")
            print(f"  FK calculada:       [{x_calc:.2f}, {y_calc:.2f}, {z_calc:.2f}]")
            print(f"  Error euclidiano:   {err:.2f} mm")

    logger.info("P1 completado")


# -----------------------------------------------------------------------
# P2 - Cinematica Directa (FK)
# -----------------------------------------------------------------------
def demo_p2(mc):
    _sep("P2 - CINEMATICA DIRECTA (FK)")
    if not P2_OK:
        print("[SKIP] p2_cinematica_directa no disponible")
        return

    results = _p2_mod.run_fk_verification(mc=mc)

    # Graficar espacio de trabajo si matplotlib disponible
    try:
        fk = _p2_mod.ForwardKinematics()
        print("\nGenerando nube de puntos XZ (5000 muestras)...")
        cloud = fk.workspace_cloud(n_samples=5000)
        print(f"  Rango X: [{cloud[:,0].min():.1f}, {cloud[:,0].max():.1f}] mm")
        print(f"  Rango Z: [{cloud[:,1].min():.1f}, {cloud[:,1].max():.1f}] mm")
        _p2_mod.plot_workspace(cloud)
    except Exception as e:
        print(f"  (Grafica omitida: {e})")

    logger.info("P2 completado")


# -----------------------------------------------------------------------
# P3 - Cinematica Inversa (IK)
# -----------------------------------------------------------------------
def demo_p3(mc):
    _sep("P3 - CINEMATICA INVERSA (IK)")
    if not P3_OK:
        print("[SKIP] p3_cinematica_inversa no disponible")
        return

    _p3_mod.compare_ik_solutions(mc=mc)

    # Mostrar configuraciones singulares
    try:
        _p3_mod.identify_singular_configs()
    except AttributeError:
        pass

    logger.info("P3 completado")


# -----------------------------------------------------------------------
# P4 - Evasion de Colisiones
# -----------------------------------------------------------------------
def demo_p4(mc):
    _sep("P4 - EVASION DE COLISIONES")
    if not P4_OK:
        print("[SKIP] p4_evasion_colisiones no disponible")
        return

    print("\nEscenarios de colision identificados en el laboratorio:")
    for k, v in _p4_mod.COLLISION_SCENARIOS.items():
        print(f"  [{k}]")
        print(f"    Causa:      {v['causa']}")
        print(f"    Prevencion: {v['prevencion']}")

    checker = _p4_mod.CollisionChecker(mc=mc)

    test_cases = [
        ([0,    0,    0,    0,    0,  -45], "HOME segura"),
        ([30,   0,    0,  -45,    0,    0], "Configuracion normal"),
        ([0,  -120,   0,    0,    0,    0], "J2 fuera de limite"),
        ([0,  -100,  50,    0,    0,    0], "Z peligrosa"),
    ]

    print(f"\n{'Descripcion':<30} {'Limites':>10} {'Altura':>12} {'Estado':>10}")
    print("-"*67)
    for angles, desc in test_cases:
        ok_l, msg_l = checker.check_joint_limits(angles)
        ok_z, z_val, msg_z = checker.check_min_height(angles)
        estado = "SEGURA" if (ok_l and ok_z) else "PELIGROSA"
        print(f"{desc:<30} {'OK' if ok_l else 'FALLA':>10} "
              f"{f'Z={z_val:.0f}mm':>12} {estado:>10}")

    # Si hay robot: demostrar safe_move con configuracion segura
    if mc is not None:
        print("\n[P4] Demostrando safe_move con HOME...")
        checker.safe_move([0, 0, 0, 0, 0, -45], speed=20)

    logger.info("P4 completado")


# -----------------------------------------------------------------------
# P5 - Control de Trayectorias (5 ciclos)
# -----------------------------------------------------------------------
def demo_p5(mc):
    _sep("P5 - CONTROL DE TRAYECTORIAS")
    if not P5_OK:
        print("[SKIP] control no disponible")
        return

    print("\nPoses clave del ciclo:")
    for nombre, angles in _p5_mod.POSES.items():
        print(f"  {nombre:<14}: {[round(a, 1) for a in angles]}")

    ctrl = _p5_mod.RobotController(mc=mc)

    if mc is not None:
        mc.set_gripper_state(0, _p5_mod.GRIPPER_SPEED)
        time.sleep(1.0)

    N = 5
    log = []
    print(f"\nEjecutando {N} ciclos consecutivos...")
    print("="*55)

    for n in range(1, N + 1):
        print(f"\n--- Ciclo {n}/{N} ---")
        result = ctrl.run_ciclo(n)
        log.append(result)
        estado = "OK" if result['exito'] else f"FALLO (paso {result['paso_fallo']})"
        print(f"  -> {estado}  ({result['tiempo']}s)")
        if not result['exito']:
            time.sleep(2.0)

    n_ok = sum(1 for r in log if r['exito'])
    tasa = n_ok / N * 100

    print("\n" + "="*55)
    print(f"{'Ciclo':<8} {'Resultado':<12} {'Tiempo(s)':>10} {'Paso fallo':>12}")
    print("-"*55)
    for r in log:
        res  = "OK" if r['exito'] else "FALLO"
        paso = "-" if r['exito'] else str(r['paso_fallo'])
        print(f"  {r['ciclo']:<6} {res:<12} {r['tiempo']:>10.2f} {paso:>12}")
    print("="*55)
    print(f"Exitosos: {n_ok}/{N}  Tasa: {tasa:.0f}%")
    print("Resultado P5:", "APROBADO" if tasa >= 80 else "REQUIERE AJUSTE")

    logger.info("P5 completado - tasa %.0f%%", tasa)


# -----------------------------------------------------------------------
# P6 - Deteccion de Objetos
# -----------------------------------------------------------------------
def demo_p6(mc):
    _sep("P6 - DETECCION DE OBJETOS (Vision)")
    if not P6_OK:
        print("[SKIP] vision no disponible")
        return

    _p6_mod.validate_detection_accuracy(mc=mc, n_positions=3)
    logger.info("P6 completado")


# -----------------------------------------------------------------------
# P7 - Pipeline End-to-End (FSM)
# -----------------------------------------------------------------------

class State(Enum):
    IDLE        = "IDLE"
    DETECTANDO  = "DETECTANDO"
    CALC_IK     = "CALC_IK"
    AGARRANDO   = "AGARRANDO"
    DEPOSITANDO = "DEPOSITANDO"
    ERROR       = "ERROR"


@dataclass
class CycleResult:
    cycle_num:    int
    object_pos:   Optional[Tuple[float, float]]
    angles:       Optional[list]
    success:      bool
    errors:       int
    timestamp:    datetime
    duration_sec: float


class PipelineController:
    """
    Maquina de estados FSM para el pipeline autonomo P7.

    Integracion:
      - P6 vision  -> detect_object(frame)
      - P3 IK      -> resolver_pdf(x, y, z)
      - P4 colision-> check_collision(angles)
      - P5 control -> pick(), place()
    """

    MAX_ERRORS = 2

    def __init__(self, mc=None):
        self.mc      = mc
        self.state   = State.IDLE
        self.errors  = 0
        self.cycle   = 0
        self.obj_pos = None   # (x_mm, y_mm)
        self.angles  = None

        # Inicializar submodulos disponibles
        self.ik        = _p3_mod.InverseKinematics()    if P3_OK else None
        self.collision = _p4_mod.CollisionChecker(mc)   if P4_OK else None
        self.ctrl      = _p5_mod.RobotController(mc)    if P5_OK else None
        self.detector  = _p6_mod.ObjectDetector('red')  if P6_OK else None

        self._log("Pipeline controller inicializado")

    def _log(self, msg, level="INFO"):
        tag = f"[P7-{self.state.value}]"
        print(f"{tag} {msg}")
        if level == "ERROR":
            logger.error(msg)
        elif level == "WARNING":
            logger.warning(msg)
        else:
            logger.info(msg)

    def _goto(self, new_state):
        self._log(f"Transicion: {self.state.value} -> {new_state.value}")
        self.state = new_state

    # ------------------------------------------------------------------
    # Estados FSM
    # ------------------------------------------------------------------
    def _idle(self):
        self._log("Ciclo iniciado -> goto init_pose")
        if self.ctrl:
            self.ctrl.goto_pose('init_pose', speed=30)
            time.sleep(1.0)
        self._goto(State.DETECTANDO)

    def _detectando(self):
        self._log("Detectando objeto (timeout=10s)...")
        if self.detector is None:
            self._log("Detector no disponible", "WARNING")
            self.errors += 1
            self._goto(State.ERROR)
            return

        try:
            import cv2
            cap = cv2.VideoCapture(0)
            deadline = time.time() + 10.0
            found = False
            while time.time() < deadline:
                ret, frame = cap.read()
                if ret and frame is not None:
                    result = self.detector.detect_object(frame)
                    if result is not None:
                        self.obj_pos = result
                        self._log(f"Objeto en ({result[0]:.1f}, {result[1]:.1f}) mm")
                        found = True
                        break
                time.sleep(0.1)
            cap.release()
        except Exception as e:
            self._log(f"Error de camara: {e}", "ERROR")
            found = False

        if found:
            self._goto(State.CALC_IK)
        else:
            self._log("Timeout: objeto no detectado", "WARNING")
            self.errors += 1
            self._goto(State.ERROR)

    def _calc_ik(self):
        if self.obj_pos is None:
            self.errors += 1
            self._goto(State.ERROR)
            return

        x_mm, y_mm = self.obj_pos
        z_mm = _p5_mod.COORDS.get('pick_z_upper', 170.0) if P5_OK else 170.0

        self._log(f"IK para ({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}) mm")

        angles = None
        if self.ik:
            try:
                angles = self.ik.resolver_pdf(x_mm, y_mm, z_mm, codo_arriba=True)
            except Exception as e:
                self._log(f"IK analitica fallo: {e}", "WARNING")

        if angles is None:
            self._log("IK no convergio - posicion fuera de workspace", "WARNING")
            self.errors += 1
            self._goto(State.ERROR)
            return

        # Verificar colisiones
        if self.collision and not self.collision.check_collision(angles):
            self._log("Configuracion con riesgo de colision -> usando SAFE_WAYPOINT", "WARNING")
            angles = _p4_mod.SAFE_WAYPOINT_ANGLES if P4_OK else angles

        self.angles = angles
        self._log(f"IK resuelta: {[round(a, 1) for a in angles]}")
        self._goto(State.AGARRANDO)

    def _agarrando(self):
        if self.obj_pos is None:
            self.errors += 1
            self._goto(State.ERROR)
            return

        x_mm, y_mm = self.obj_pos
        self._log(f"Agarrando en ({x_mm:.1f}, {y_mm:.1f})")

        if self.ctrl:
            ok = self.ctrl.pick(x_mm, y_mm)
            if not ok:
                self.errors += 1
                self._goto(State.ERROR)
                return
            time.sleep(0.5)

        self._log("Agarre exitoso")
        self._goto(State.DEPOSITANDO)

    def _depositando(self):
        self._log("Depositando objeto en zona B")

        if self.ctrl:
            ok = self.ctrl.place()
            if not ok:
                self.errors += 1
                self._goto(State.ERROR)
                return
            time.sleep(0.5)

        self._log("Deposito exitoso")
        self._goto(State.IDLE)

    def _error(self):
        self._log(f"Manejando error ({self.errors}/{self.MAX_ERRORS})")

        if self.errors >= self.MAX_ERRORS:
            self._log("Maximo de errores alcanzado - abandonando ciclo", "WARNING")
            self.errors = 0
            self._goto(State.IDLE)
            return

        # Ir a waypoint seguro
        if P4_OK and self.mc is not None:
            try:
                self.mc.send_angles(_p4_mod.SAFE_WAYPOINT_ANGLES, 30)
                time.sleep(1.5)
            except Exception:
                pass

        if self.ctrl:
            try:
                self.ctrl.goto_pose('init_pose', speed=30)
                time.sleep(1.0)
            except Exception:
                pass

        self._goto(State.IDLE)

    # ------------------------------------------------------------------
    # Loop de un ciclo completo
    # ------------------------------------------------------------------
    def run_cycle(self):
        """Ejecuta un ciclo FSM completo. Retorna CycleResult."""
        t0 = time.time()
        self.cycle  += 1
        self.errors  = 0
        self.obj_pos = None
        self.angles  = None

        _sep_inner(f"CICLO {self.cycle}")

        _handlers = {
            State.IDLE:        self._idle,
            State.DETECTANDO:  self._detectando,
            State.CALC_IK:     self._calc_ik,
            State.AGARRANDO:   self._agarrando,
            State.DEPOSITANDO: self._depositando,
            State.ERROR:       self._error,
        }

        for _ in range(50):   # limite de iteraciones por ciclo
            _handlers[self.state]()
            if self.state == State.IDLE and _ > 0:
                break
            time.sleep(0.3)

        duration = time.time() - t0
        success  = (self.state == State.IDLE and self.errors == 0)

        result = CycleResult(
            cycle_num    = self.cycle,
            object_pos   = self.obj_pos,
            angles       = self.angles,
            success      = success,
            errors       = self.errors,
            timestamp    = datetime.now(),
            duration_sec = round(duration, 2),
        )
        self._log(f"Ciclo {self.cycle}: {'EXITO' if success else 'FALLO'} "
                  f"(dur={duration:.1f}s errores={self.errors})")
        return result


# -----------------------------------------------------------------------
# Helpers de formato
# -----------------------------------------------------------------------
def _sep(title):
    print("\n" + "="*65)
    print(f"  {title}")
    print("="*65)


def _sep_inner(title):
    print("\n" + "-"*55)
    print(f"  {title}")
    print("-"*55)


# -----------------------------------------------------------------------
# main()
# -----------------------------------------------------------------------
def main():
    global _mc_global

    _sep("PIPELINE UNIFICADO MyCobot 280  (P1 -> P7)")
    logger.info("="*60)
    logger.info("INICIO PIPELINE UNIFICADO")
    logger.info("="*60)

    # Conectar robot
    mc = connect_robot()
    _mc_global = mc

    # ----------------------------------------------------------------
    # P1 - Tabla DH y verificacion HOME
    # ----------------------------------------------------------------
    demo_p1(mc)

    # ----------------------------------------------------------------
    # P2 - Verificacion FK (5 configuraciones)
    # ----------------------------------------------------------------
    demo_p2(mc)

    # ----------------------------------------------------------------
    # P3 - Comparacion IK analitica vs API
    # ----------------------------------------------------------------
    demo_p3(mc)

    # ----------------------------------------------------------------
    # P4 - Verificacion de colisiones y escenarios
    # ----------------------------------------------------------------
    demo_p4(mc)

    # ----------------------------------------------------------------
    # P5 - Ciclo de agarre (5 ciclos consecutivos)
    # ----------------------------------------------------------------
    demo_p5(mc)

    # ----------------------------------------------------------------
    # P6 - Deteccion de objetos y precision
    # ----------------------------------------------------------------
    demo_p6(mc)

    # ----------------------------------------------------------------
    # P7 - Pipeline end-to-end (FSM autonomo, 5 ciclos)
    # ----------------------------------------------------------------
    _sep("P7 - PIPELINE END-TO-END (FSM AUTONOMO)")

    pipeline = PipelineController(mc=mc)
    results  = []
    NUM_CYCLES = 5

    try:
        for _ in range(NUM_CYCLES):
            r = pipeline.run_cycle()
            results.append(r)
            time.sleep(2.0)

    except KeyboardInterrupt:
        logger.warning("Pipeline interrumpido por usuario")
        print("\n[WARN] Pipeline interrumpido")

    # ----------------------------------------------------------------
    # Resumen final
    # ----------------------------------------------------------------
    _sep("RESUMEN DE EJECUCION")

    total     = len(results)
    n_ok      = sum(1 for r in results if r.success)
    n_errors  = sum(r.errors for r in results)
    avg_dur   = (sum(r.duration_sec for r in results) / total) if total else 0.0
    tasa      = (n_ok / total * 100) if total else 0.0

    print(f"{'Ciclo':<8} {'Estado':<10} {'Dur(s)':>8} {'Errores':>8} {'Objeto(mm)':>20}")
    print("-"*58)
    for r in results:
        estado = "EXITO" if r.success else "FALLO"
        pos_str = (f"({r.object_pos[0]:.0f}, {r.object_pos[1]:.0f})"
                   if r.object_pos else "N/A")
        print(f"  {r.cycle_num:<6} {estado:<10} {r.duration_sec:>8.1f} "
              f"{r.errors:>8} {pos_str:>20}")

    print("="*58)
    print(f"Total ciclos  : {total}")
    print(f"Exitosos      : {n_ok}/{total}  ({tasa:.0f}%)")
    print(f"Errores total : {n_errors}")
    print(f"Duracion prom : {avg_dur:.1f}s/ciclo")
    print(f"Log guardado  : {LOG_FILE}")
    print("Resultado P7  :", "APROBADO" if tasa >= 80 else "REQUIERE AJUSTE")
    print("="*58)

    logger.info("RESUMEN: %d/%d ciclos OK (%.0f%%), prom=%.1fs", n_ok, total, tasa, avg_dur)
    logger.info("FIN PIPELINE")


if __name__ == "__main__":
    main()
