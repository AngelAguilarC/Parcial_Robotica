
import sys
import os
import time
import logging
from math import sqrt

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'P1_DH'))
from p1_dh_representacion import forward_kinematics, extract_position

logger = logging.getLogger(__name__)

# -----------------------------------------------------------------------
# Escenarios de colision identificados en el laboratorio real
# -----------------------------------------------------------------------
COLLISION_SCENARIOS = {
    "colision_mesa": {
        "descripcion": "El extremo o los eslabones impactan la superficie de la mesa.",
        "causa": "Angulos J2/J3 que llevan el brazo por debajo de Z=50mm.",
        "prevencion": "Verificar Z del extremo via FK antes de send_angles().",
    },
    "auto_colision_J2_J3": {
        "descripcion": "El eslabon del codo golpea el eslabon del hombro.",
        "causa": "J3 en valores extremos con J2 en posicion critica.",
        "prevencion": "Restringir J3 a [-120, 120] y J2 a [-110, 70].",
    },
    "colision_camara": {
        "descripcion": "El gripper choca con la camara montada en el robot.",
        "causa": "Movimientos rapidos con J4 en angulos extremos.",
        "prevencion": "Pasar por waypoint seguro entre pick y place.",
    },
    "colision_zona_deposito": {
        "descripcion": "El brazo choca con los contenedores de colores en la mesa.",
        "causa": "Trayectorias directas que pasan por encima de los contenedores.",
        "prevencion": "Elevar Z a 200mm antes de desplazarse lateralmente.",
    },
}

# -----------------------------------------------------------------------
# Limites conservadores (mas restrictivos que los fisicos)
# -----------------------------------------------------------------------
CONSERVATIVE_JOINT_LIMITS = {
    # Joint: (min_deg, max_deg)  [rangos fisicos entre parentesis]
    0: (-160, 160),   # J1 fisico: -168 a 168
    1: (-110,  70),   # J2 fisico: -135 a  90  <-- mas restrictivo
    2: (-120, 120),   # J3 fisico: -150 a 150  <-- mas restrictivo
    3: (-130, 130),   # J4 fisico: -145 a 145
    4: (-150, 150),   # J5 fisico: -165 a 165
    5: (-175, 175),   # J6 fisico: -180 a 180
}

Z_MIN_SAFE = 60.0      # mm - altura minima del extremo sobre la mesa
Z_CLEARANCE = 200.0    # mm - altura de clearance para waypoints seguros

# Waypoint intermedio seguro (angulos) entre cualquier pick y cualquier place
SAFE_WAYPOINT_ANGLES = [0.0, 0.0, -90.0, 95.0, 0.0, -45.0]


class CollisionChecker:
    """
    Verifica colisiones potenciales antes de ejecutar movimientos del robot.

    Mecanismos implementados:
      1. check_joint_limits()  - verifica limites conservadores por joint
      2. check_min_height()    - verifica Z minimo del extremo via FK
      3. safe_move()           - ejecuta movimiento con waypoint de clearance
    """

    def __init__(self, mc=None):
        self.mc = mc
        self.joint_limits = CONSERVATIVE_JOINT_LIMITS
        self.z_min = Z_MIN_SAFE
        self.z_clearance = Z_CLEARANCE

    # ------------------------------------------------------------------
    # Mecanismo 1: limites conservadores
    # ------------------------------------------------------------------
    def check_joint_limits(self, angles):
        """
        Verifica que todos los angulos esten dentro de los limites conservadores.

        Parametros
        ----------
        angles : list[float]  6 angulos en grados

        Retorna
        -------
        (bool, str)  (valido, mensaje de error si invalido)
        """
        for i, angle in enumerate(angles):
            lo, hi = self.joint_limits[i]
            if not (lo <= angle <= hi):
                msg = (f"J{i+1}={angle:.1f} deg fuera de limite "
                       f"[{lo}, {hi}] deg")
                return False, msg
        return True, "Todos los joints dentro de limites"

    # ------------------------------------------------------------------
    # Mecanismo 2: verificacion de altura minima por FK
    # ------------------------------------------------------------------
    def check_min_height(self, angles):
        """
        Calcula la posicion Z del extremo via FK y verifica que sea >= Z_MIN_SAFE.

        Parametros
        ----------
        angles : list[float]  6 angulos en grados

        Retorna
        -------
        (bool, float, str)  (valido, z_calculado, mensaje)
        """
        T = forward_kinematics(angles)
        _, _, z = extract_position(T)
        if z < self.z_min:
            msg = f"Altura calculada Z={z:.1f} mm < Z_min={self.z_min:.1f} mm"
            return False, z, msg
        return True, z, f"Altura Z={z:.1f} mm OK (>= {self.z_min:.1f} mm)"

    # ------------------------------------------------------------------
    # Mecanismo 3: movimiento seguro con waypoint de clearance
    # ------------------------------------------------------------------
    def safe_move(self, target_angles, speed=40):
        """
        Ejecuta un movimiento seguro pasando por el waypoint de clearance.

        Secuencia:
          1. Verificar limites de target_angles
          2. Verificar altura minima de target_angles
          3. Subir al waypoint de clearance
          4. Ir al destino

        Parametros
        ----------
        target_angles : list[float]  angulos objetivo en grados
        speed         : int          velocidad (0-100)

        Retorna
        -------
        bool  True si el movimiento fue ejecutado, False si fue bloqueado
        """
        # Paso 1: verificar limites
        valid_limits, msg_limits = self.check_joint_limits(target_angles)
        if not valid_limits:
            logger.warning(f"[CollisionChecker] Movimiento BLOQUEADO: {msg_limits}")
            return False

        # Paso 2: verificar altura minima
        valid_z, z_val, msg_z = self.check_min_height(target_angles)
        if not valid_z:
            logger.warning(f"[CollisionChecker] Movimiento BLOQUEADO: {msg_z}")
            return False

        # Paso 3 y 4: ejecutar con waypoint si hay hardware
        if self.mc is not None:
            logger.info("[CollisionChecker] Subiendo a waypoint de clearance...")
            self.mc.send_angles(SAFE_WAYPOINT_ANGLES, speed)
            time.sleep(2.5)

            logger.info(f"[CollisionChecker] Moviendo a destino (Z={z_val:.1f} mm)...")
            self.mc.send_angles(target_angles, speed)
            time.sleep(2.5)
        else:
            logger.info(f"[CollisionChecker] [OFFLINE] Movimiento validado: Z={z_val:.1f} mm")

        return True

    def safe_coords(self, target_coords, speed=40):
        """
        Ejecuta movimiento cartesiano seguro elevando Z antes del desplazamiento.

        Parametros
        ----------
        target_coords : list[float]  [x,y,z,rx,ry,rz] en mm/grados
        speed         : int          velocidad (0-100)

        Retorna
        -------
        bool  True si ejecutado, False si bloqueado
        """
        x, y, z = target_coords[0], target_coords[1], target_coords[2]

        if z < self.z_min:
            logger.warning(f"[CollisionChecker] Coords BLOQUEADAS: Z={z:.1f} < Z_min={self.z_min:.1f}")
            return False

        if self.mc is not None:
            # Elevar a clearance primero
            current = self.mc.get_coords()
            if current and len(current) >= 6:
                elevated = list(current)
                elevated[2] = self.z_clearance
                logger.info(f"[CollisionChecker] Elevando a Z={self.z_clearance:.0f} mm")
                self.mc.send_coords(elevated, speed, 1)
                time.sleep(2.0)

            logger.info(f"[CollisionChecker] Moviendo a [{x:.1f},{y:.1f},{z:.1f}]")
            self.mc.send_coords(target_coords, speed, 1)
            time.sleep(2.5)
        else:
            logger.info(f"[CollisionChecker] [OFFLINE] Coords validadas: Z={z:.1f} mm OK")

        return True


def validate_with_real_robot(mc):
    """
    Ejecuta una secuencia de prueba en el robot real para validar
    que no ocurren colisiones durante el pipeline.
    """
    checker = CollisionChecker(mc=mc)

    test_sequences = [
        # (descripcion, angles)
        ("Pose home",          [  0,   0,   0,   0,   0, -45]),
        ("Watch pose",         [ 42,   0,   0, -85,  -7,  -3]),
        ("Config limite J2",   [  0, -110,  0,   0,   0,   0]),
        ("Config fuera limite", [0, -120,  0,   0,   0,   0]),   # debe ser bloqueada
        ("Config baja Z",      [ 0,  90,  90,  90,   0,   0]),   # puede ser bloqueada
    ]

    print("\n" + "="*70)
    print("VALIDACION EVASION DE COLISIONES - Robot real")
    print("="*70)
    for desc, angles in test_sequences:
        print(f"\n[{desc}] -> angles={angles}")
        valid_l, msg_l = checker.check_joint_limits(angles)
        valid_z, z_val, msg_z = checker.check_min_height(angles)
        print(f"  Limites: {'OK' if valid_l else 'BLOQUEADO'} - {msg_l}")
        print(f"  Altura:  {'OK' if valid_z else 'BLOQUEADO'} - {msg_z}")

        if valid_l and valid_z:
            result = checker.safe_move(angles)
            print(f"  Movimiento ejecutado: {result}")
        else:
            print(f"  Movimiento bloqueado por CollisionChecker.")

    mc.send_angles([0, 0, 0, 0, 0, -45], 40)
    time.sleep(2)
    print("\nValidacion completada. Robot en pose home.")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s - %(levelname)s - %(message)s')

    print("="*70)
    print("P4 - EVASION DE COLISIONES - MyCobot 280")
    print("="*70)

    print("\nEscenarios de colision identificados en el laboratorio:")
    for name, info in COLLISION_SCENARIOS.items():
        print(f"\n[{name}]")
        print(f"  Descripcion: {info['descripcion']}")
        print(f"  Causa:       {info['causa']}")
        print(f"  Prevencion:  {info['prevencion']}")

    print("\nLimites conservadores por joint:")
    for i, (lo, hi) in CONSERVATIVE_JOINT_LIMITS.items():
        print(f"  J{i+1}: [{lo:>5}, {hi:>5}] deg")

    # Pruebas offline
    checker = CollisionChecker(mc=None)
    test_cases = [
        ([  0,  0,  0,  0,  0, -45], "Pose home (debe pasar)"),
        ([  0, -120, 0, 0,  0,   0], "J2=-120 (debe ser bloqueado)"),
        ([  0,  80, 80, 90, 0,   0], "Z muy bajo (debe ser bloqueado)"),
    ]
    print("\nPruebas de verificacion offline:")
    for angles, desc in test_cases:
        valid_l, msg_l = checker.check_joint_limits(angles)
        valid_z, z_val, msg_z = checker.check_min_height(angles)
        status = "PASO" if (valid_l and valid_z) else "BLOQUEADO"
        print(f"  [{status}] {desc}")
        if not valid_l:
            print(f"    -> {msg_l}")
        if not valid_z:
            print(f"    -> {msg_z}")

    # Para validacion con hardware:
    # from pymycobot.mycobot import MyCobot
    # mc = MyCobot('/dev/ttyUSB0', 1000000)
    # mc.power_on()
    # time.sleep(0.5)
    # validate_with_real_robot(mc)
