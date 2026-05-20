#!/usr/bin/env python3
# coding: utf-8
"""
P7 - Pipeline End-to-End
MyCobot 280 - Líder de Integración

Máquina de estados (FSM):
  IDLE        → goto INIT_POSE + WATCH_POSE
  DETECTANDO  → detect_object(frame) con timeout 10s
  CALC_IK     → ik_solve(x, y, z) + CollisionChecker
  AGARRANDO   → pick(x, y, z)
  DEPOSITANDO → place()
  ERROR       → goto SAFE_TRANSIT + INIT_POSE (máximo 2 errores)

Integra:
  - Visión (P6):      detect_object(frame) -> (x_mm, y_mm)
  - Cinemática (P3):  ik_solve(x, y, z)   -> angles
  - Control (P5):     pick(), place(), goto_pose()

Ejecuta 5 ciclos autónomos completos con logging detallado.
"""

import sys
import os
import logging
import time
import cv2
from datetime import datetime
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Tuple

# -----------------------------------------------------------------------
# Configurar rutas para importar módulos de otras prácticas
# -----------------------------------------------------------------------
CINEMATICA_DIR = os.path.join(os.path.dirname(__file__), '..', 'cinematica')
P1_DIR = os.path.join(os.path.dirname(__file__), '..', 'P1_DH')
P2_DIR = os.path.join(os.path.dirname(__file__), '..', 'P2_FK')
P3_DIR = os.path.join(os.path.dirname(__file__), '..', 'P3_IK')
P4_DIR = os.path.join(os.path.dirname(__file__), '..', 'P4_COLISIONES')
P5_DIR = os.path.join(os.path.dirname(__file__), '..', 'P5_CONTROL')
P6_DIR = os.path.join(os.path.dirname(__file__), '..', 'P6_VISION')

sys.path.insert(0, CINEMATICA_DIR)
sys.path.insert(0, P1_DIR)
sys.path.insert(0, P2_DIR)
sys.path.insert(0, P3_DIR)
sys.path.insert(0, P4_DIR)
sys.path.insert(0, P5_DIR)
sys.path.insert(0, P6_DIR)

# -----------------------------------------------------------------------
# Importar módulos de prácticas previas (P1-P6)
# -----------------------------------------------------------------------

# P1 - Cinemática (DH)
try:
    from p1_dh_representacion import DH_TABLE, dh_matrix, forward_kinematics, extract_position
    P1_AVAILABLE = True
except ImportError as e:
    print(f"[WARN] P1 (DH) no disponible: {e}")
    P1_AVAILABLE = False

# P2 - Cinemática Directa (FK)
try:
    from p2_cinematica_directa import ForwardKinematics
    P2_AVAILABLE = True
except ImportError as e:
    print(f"[WARN] P2 (FK) no disponible: {e}")
    P2_AVAILABLE = False

# P3 - Cinemática Inversa (IK)
try:
    from p3_cinematica_inversa import InverseKinematics, compare_ik_solutions
    P3_AVAILABLE = True
except ImportError as e:
    print(f"[WARN] P3 (IK) no disponible: {e}")
    P3_AVAILABLE = False

# P4 - Evasión de Colisiones
try:
    from p4_evasion_colisiones import CollisionChecker, SAFE_WAYPOINT_ANGLES, COLLISION_SCENARIOS
    P4_AVAILABLE = True
except ImportError as e:
    print(f"[WARN] P4 (Colisiones) no disponible: {e}")
    P4_AVAILABLE = False

# P5 - Control de Trayectorias
try:
    from control import RobotController, POSES, COORDS, GRIPPER_OPEN, GRIPPER_CLOSED
    P5_AVAILABLE = True
except ImportError as e:
    print(f"[WARN] P5 (Control) no disponible: {e}")
    P5_AVAILABLE = False

# P6 - Detección de Objetos
try:
    from vision import ObjectDetector, pixel_to_mm
    P6_AVAILABLE = True
except ImportError as e:
    print(f"[WARN] P6 (Visión) no disponible: {e}")
    P6_AVAILABLE = False

print(f"\n[MAIN] Módulos disponibles: P1={P1_AVAILABLE}, P2={P2_AVAILABLE}, P3={P3_AVAILABLE}, "
      f"P4={P4_AVAILABLE}, P5={P5_AVAILABLE}, P6={P6_AVAILABLE}")

# -----------------------------------------------------------------------
# Configurar logging
# -----------------------------------------------------------------------
LOG_FILE = os.path.join(os.path.dirname(__file__), 'sesion.log')
logging.basicConfig(
    filename=LOG_FILE,
    level=logging.INFO,
    format='%(asctime)s - [MAIN] - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

# -----------------------------------------------------------------------
# Definir máquina de estados (FSM)
# -----------------------------------------------------------------------

class State(Enum):
    """Estados de la máquina de estados."""
    IDLE = "IDLE"
    DETECTANDO = "DETECTANDO"
    CALC_IK = "CALC_IK"
    AGARRANDO = "AGARRANDO"
    DEPOSITANDO = "DEPOSITANDO"
    ERROR = "ERROR"


@dataclass
class CycleResult:
    """Resultado de un ciclo completo de pick-and-place."""
    cycle_num: int
    object_pos: Optional[Tuple[float, float]]  # (x_mm, y_mm)
    angles: Optional[list]  # [J1, J2, J3, J4, J5, J6]
    success: bool
    errors: int
    timestamp: datetime
    duration_sec: float


class PipelineController:
    """Controlador FSM para el pipeline de pick-and-place autónomo."""
    
    def __init__(self, robot=None, logging_enabled=True):
        """
        Inicializa el controlador de pipeline.
        
        Args:
            robot: Instancia de MyCobot (si None, modo simulación)
            logging_enabled: Si True, registra en sesion.log
        """
        self.robot = robot
        self.logging_enabled = logging_enabled
        self.state = State.IDLE
        self.error_count = 0
        self.MAX_ERRORS_PER_CYCLE = 2
        self.object_pos = None
        self.angles = None
        self.cycle_num = 0
        
        # Inicializar módulos opcionales
        self.ik_solver = None
        self.collision_checker = None
        self.robot_controller = None
        self.detector = None
        
        if P3_AVAILABLE:
            try:
                self.ik_solver = InverseKinematics()
                self.log("IK solver inicializado")
            except Exception as e:
                self.log(f"Error inicializando IK solver: {e}", level="ERROR")
        
        if P4_AVAILABLE:
            try:
                self.collision_checker = CollisionChecker()
                self.log("Collision checker inicializado")
            except Exception as e:
                self.log(f"Error inicializando collision checker: {e}", level="ERROR")
        
        if P5_AVAILABLE:
            try:
                self.robot_controller = RobotController(mc=robot)
                self.log("Robot controller inicializado")
            except Exception as e:
                self.log(f"Error inicializando robot controller: {e}", level="ERROR")
        
        if P6_AVAILABLE:
            try:
                self.detector = ObjectDetector(target_color='red')
                self.log("Object detector inicializado")
            except Exception as e:
                self.log(f"Error inicializando detector: {e}", level="ERROR")
    
    def log(self, message, level="INFO"):
        """Registra mensaje en sesion.log."""
        if self.logging_enabled:
            if level == "ERROR":
                logger.error(message)
            elif level == "WARNING":
                logger.warning(message)
            else:
                logger.info(message)
        print(f"[MAIN-{self.state.value}] {message}")
    
    def transition_to(self, new_state):
        """Transición a un nuevo estado."""
        self.log(f"Transición: {self.state.value} → {new_state.value}")
        self.state = new_state
    
    def state_idle(self):
        """
        Estado IDLE: Ir a pose inicial y esperar.
        Transición: → DETECTANDO
        """
        try:
            self.log("Iniciando ciclo: IDLE → goto INIT_POSE")
            
            if self.robot_controller:
                self.robot_controller.goto_pose('init_pose', speed=30)
                time.sleep(1.0)
            
            self.transition_to(State.DETECTANDO)
        except Exception as e:
            self.log(f"Error en IDLE: {e}", level="ERROR")
            self.error_count += 1
            self.transition_to(State.ERROR)
    
    def state_detectando(self):
        """
        Estado DETECTANDO: Detectar objeto en escena.
        Transición: → CALC_IK (si detecta) o → ERROR (si timeout)
        """
        try:
            self.log("Detectando objeto (timeout=10s)...")
            
            if not self.detector:
                self.log("Detector no disponible", level="WARNING")
                self.transition_to(State.ERROR)
                return
            
            # Capturar frame de cámara
            cap = cv2.VideoCapture(0)
            frame = None
            start_time = time.time()
            timeout = 10.0
            
            while (time.time() - start_time) < timeout:
                ret, frame = cap.read()
                if ret and frame is not None:
                    # Detectar objeto (color configurado, ej: "red")
                    result = self.detector.detect_object(frame)
                    if result is not None:
                        x_mm, y_mm = result
                        self.object_pos = (x_mm, y_mm)
                        self.log(f"Objeto detectado: ({x_mm:.1f}, {y_mm:.1f}) mm")
                        cap.release()
                        self.transition_to(State.CALC_IK)
                        return
                time.sleep(0.1)
            
            cap.release()
            self.log("Timeout en detección", level="WARNING")
            self.error_count += 1
            self.transition_to(State.ERROR)
        
        except Exception as e:
            self.log(f"Error en DETECTANDO: {e}", level="ERROR")
            self.error_count += 1
            self.transition_to(State.ERROR)
    
    def state_calc_ik(self):
        """
        Estado CALC_IK: Calcular IK para posición detectada.
        Transición: → AGARRANDO (si OK) o → ERROR (si no convergente)
        """
        try:
            if not self.object_pos:
                self.log("No hay posición de objeto", level="ERROR")
                self.error_count += 1
                self.transition_to(State.ERROR)
                return
            
            x_mm, y_mm = self.object_pos
            z_mm = COORDS.get('pick_z_upper', 170)  # Altura de aproximación
            
            self.log(f"Calculando IK para posición: ({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f})")
            
            if not self.ik_solver:
                self.log("IK solver no disponible", level="WARNING")
                # Si no hay solver, simplemente ir a pick directamente
                self.transition_to(State.AGARRANDO)
                return
            
            # Resolver IK con método analítico
            try:
                angles = self.ik_solver.resolver_pdf(x_mm, y_mm, z_mm, codo_arriba=True)
            except:
                angles = None
            
            if angles is None:
                self.log("IK no convergió (posición fuera de workspace)", level="WARNING")
                self.error_count += 1
                self.transition_to(State.ERROR)
                return
            
            # Verificar colisiones
            if self.collision_checker:
                try:
                    is_safe = self.collision_checker.check_collision(angles)
                    if not is_safe:
                        self.log("Ruta en colisión, intentando waypoint seguro", level="WARNING")
                        angles = SAFE_WAYPOINT_ANGLES
                except:
                    pass
            
            self.angles = angles
            self.log(f"IK resuelto: {[f'{a:.1f}' for a in angles]}")
            self.transition_to(State.AGARRANDO)
        
        except Exception as e:
            self.log(f"Error en CALC_IK: {e}", level="ERROR")
            self.error_count += 1
            self.transition_to(State.ERROR)
    
    def state_agarrando(self):
        """
        Estado AGARRANDO: Ejecutar movimiento de agarre.
        Transición: → DEPOSITANDO (si OK) o → ERROR (si falla)
        """
        try:
            if not self.object_pos:
                self.log("No hay posición de objeto", level="ERROR")
                self.error_count += 1
                self.transition_to(State.ERROR)
                return
            
            x_mm, y_mm = self.object_pos
            self.log(f"Agarrando en posición: ({x_mm:.1f}, {y_mm:.1f})")
            
            if self.robot_controller:
                ok = self.robot_controller.pick(x_mm, y_mm, rx=-175.0, ry=0.0, rz=-45.0)
                if not ok:
                    self.log("Fallo en pick", level="WARNING")
                    self.error_count += 1
                    self.transition_to(State.ERROR)
                    return
                time.sleep(1.0)
            
            self.log("Objeto agarrado exitosamente")
            self.transition_to(State.DEPOSITANDO)
        
        except Exception as e:
            self.log(f"Error en AGARRANDO: {e}", level="ERROR")
            self.error_count += 1
            self.transition_to(State.ERROR)
    
    def state_depositando(self):
        """
        Estado DEPOSITANDO: Depositar objeto en zona de descarga.
        Transición: → IDLE (siguiente ciclo)
        """
        try:
            self.log("Depositando objeto")
            
            if self.robot_controller:
                ok = self.robot_controller.place(place_name='place_pose')
                if not ok:
                    self.log("Fallo en place", level="WARNING")
                    self.error_count += 1
                    self.transition_to(State.ERROR)
                    return
                time.sleep(1.0)
            
            self.log("Objeto depositado exitosamente")
            self.transition_to(State.IDLE)
        
        except Exception as e:
            self.log(f"Error en DEPOSITANDO: {e}", level="ERROR")
            self.error_count += 1
            self.transition_to(State.ERROR)
    
    def state_error(self):
        """
        Estado ERROR: Manejo de errores.
        Si error_count < MAX: ir a SAFE_WAYPOINT → INIT_POSE → IDLE
        Si error_count >= MAX: abandonar ciclo, ir a IDLE
        """
        try:
            self.log(f"Manejando error (count={self.error_count}/{self.MAX_ERRORS_PER_CYCLE})")
            
            if self.error_count >= self.MAX_ERRORS_PER_CYCLE:
                self.log(f"Máximo número de errores alcanzado, abandonando ciclo", level="WARNING")
                self.error_count = 0
                self.transition_to(State.IDLE)
                return
            
            # Ir a waypoint seguro
            if self.robot_controller and SAFE_WAYPOINT_ANGLES:
                try:
                    self.log(f"Moviendo a waypoint seguro: {SAFE_WAYPOINT_ANGLES}")
                    if hasattr(self.robot_controller, 'mc') and self.robot_controller.mc:
                        self.robot_controller.mc.send_angles(SAFE_WAYPOINT_ANGLES, 30)
                        time.sleep(1.0)
                except:
                    pass
            
            # Volver a pose inicial
            if self.robot_controller:
                try:
                    self.robot_controller.goto_pose('init_pose', speed=30)
                    time.sleep(1.0)
                except:
                    pass
            
            self.transition_to(State.IDLE)
        
        except Exception as e:
            self.log(f"Error crítico en ERROR handler: {e}", level="ERROR")
            # Forzar retorno a IDLE
            self.error_count = 0
            self.transition_to(State.IDLE)
    
    def run_cycle(self):
        """Ejecuta un ciclo completo (pick-and-place)."""
        start_time = time.time()
        self.cycle_num += 1
        self.error_count = 0
        self.object_pos = None
        self.angles = None
        
        self.log(f"\n{'='*60}")
        self.log(f"INICIANDO CICLO {self.cycle_num}")
        self.log(f"{'='*60}")
        
        # Loop FSM hasta completar ciclo o error crítico
        max_iterations = 100
        iteration = 0
        
        while iteration < max_iterations:
            iteration += 1
            
            # Ejecutar estado actual
            if self.state == State.IDLE:
                self.state_idle()
            elif self.state == State.DETECTANDO:
                self.state_detectando()
            elif self.state == State.CALC_IK:
                self.state_calc_ik()
            elif self.state == State.AGARRANDO:
                self.state_agarrando()
            elif self.state == State.DEPOSITANDO:
                self.state_depositando()
            elif self.state == State.ERROR:
                self.state_error()
            
            # Si volvemos a IDLE después de DEPOSITANDO = ciclo completado
            if self.state == State.IDLE and iteration > 1:
                break
            
            time.sleep(0.5)
        
        duration = time.time() - start_time
        success = (self.state == State.IDLE and self.error_count == 0)
        
        result = CycleResult(
            cycle_num=self.cycle_num,
            object_pos=self.object_pos,
            angles=self.angles,
            success=success,
            errors=self.error_count,
            timestamp=datetime.now(),
            duration_sec=duration
        )
        
        self.log(f"CICLO {self.cycle_num} COMPLETADO: {'✓ ÉXITO' if success else '✗ FALLÓ'} "
                 f"(duración={duration:.1f}s, errores={self.error_count})")
        self.log(f"{'='*60}\n")
        
        return result


def main():
    """Función principal: conecta al robot y ejecuta 5 ciclos autónomos."""
    
    print("\n" + "="*70)
    print("P7 - PIPELINE END-TO-END (MyCobot 280)")
    print("="*70)
    
    logger.info("="*70)
    logger.info("INICIANDO PIPELINE P7")
    logger.info("="*70)
    
    # Conectar al robot (opcional)
    robot = None
    try:
        from pymycobot.mycobot import MyCobot
        robot = MyCobot('/dev/ttyUSB0', 1000000)
        logger.info("Robot conectado: MyCobot 280")
        print("[MAIN] Robot conectado: MyCobot 280")
    except Exception as e:
        logger.warning(f"No se pudo conectar al robot: {e}")
        print(f"[WARN] Modo simulación (robot no disponible)")
    
    # Crear controlador FSM
    try:
        controller = PipelineController(robot=robot, logging_enabled=True)
        controller.log("Pipeline controller inicializado")
    except Exception as e:
        logger.error(f"Error inicializando controller: {e}")
        print(f"[ERROR] No se pudo inicializar controller: {e}")
        return
    
    # Ejecutar 5 ciclos
    results = []
    NUM_CYCLES = 5
    
    try:
        for cycle in range(1, NUM_CYCLES + 1):
            result = controller.run_cycle()
            results.append(result)
            time.sleep(2.0)  # Pausa entre ciclos
        
        # Resumen final
        logger.info("\n" + "="*70)
        logger.info("RESUMEN DE EJECUCIÓN")
        logger.info("="*70)
        
        total_cycles = len(results)
        successful_cycles = sum(1 for r in results if r.success)
        total_errors = sum(r.errors for r in results)
        avg_duration = sum(r.duration_sec for r in results) / total_cycles if total_cycles > 0 else 0
        
        logger.info(f"Total de ciclos ejecutados: {total_cycles}")
        logger.info(f"Ciclos exitosos: {successful_cycles}/{total_cycles}")
        logger.info(f"Tasa de éxito: {100*successful_cycles/total_cycles:.1f}%")
        logger.info(f"Errores totales: {total_errors}")
        logger.info(f"Duración promedio por ciclo: {avg_duration:.1f}s")
        logger.info("="*70 + "\n")
        
        print("\n" + "="*70)
        print("RESUMEN DE EJECUCIÓN")
        print("="*70)
        print(f"Total de ciclos: {total_cycles}")
        print(f"Ciclos exitosos: {successful_cycles}/{total_cycles}")
        print(f"Tasa de éxito: {100*successful_cycles/total_cycles:.1f}%")
        print(f"Errores totales: {total_errors}")
        print(f"Duración promedio: {avg_duration:.1f}s")
        print(f"Log guardado en: {LOG_FILE}")
        print("="*70 + "\n")
    
    except KeyboardInterrupt:
        logger.warning("Pipeline interrumpido por usuario")
        print("\n[WARN] Pipeline interrumpido")
    except Exception as e:
        logger.error(f"Error durante ejecución: {e}")
        print(f"\n[ERROR] Error durante ejecución: {e}")


if __name__ == "__main__":
    main()
