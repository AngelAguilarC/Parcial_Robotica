# Examen Grupal - MyCobot 280
## Cinematica, Control y Vision de Robots - ROB 2026

Robot: Yahboom MyCobot 280 (6-DOF serial)  
Puerto: `/dev/ttyUSB0` | Baud rate: 1 000 000  
Entorno: Ubuntu 20.04 + ROS 2 Foxy + Jetson Nano

---

## Estructura del Proyecto

```
EXAMEN_MYCOBOT280/
|
+-- cinematica/            # Modulo compartido (todas las clases)
|   +-- cinematica.py      # ForwardKinematics, InverseKinematics, CollisionChecker, ik_solve()
|
+-- P1_DH/                 # P1: Representacion DH
|   +-- p1_dh_representacion.py
|
+-- P2_FK/                 # P2: Cinematica Directa
|   +-- p2_cinematica_directa.py
|
+-- P3_IK/                 # P3: Cinematica Inversa
|   +-- p3_cinematica_inversa.py
|
+-- P4_COLISIONES/         # P4: Evasion de Colisiones
|   +-- p4_evasion_colisiones.py
|
+-- P5_CONTROL/            # P5: Control de Trayectorias
|   +-- control.py
|
+-- P6_VISION/             # P6: Deteccion de Objetos
|   +-- vision.py
|
+-- P7_PIPELINE/           # P7: Pipeline End-to-End
|   +-- main.py
|
+-- README.md              # Este archivo
```

---

## Resumen de Problemas

| # | Archivo principal | Clase/Funcion clave | Descripcion |
|---|-------------------|---------------------|-------------|
| P1 | `P1_DH/p1_dh_representacion.py` | `dh_matrix()`, `build_all_Ti()` | Parametros DH y matrices Ti |
| P2 | `P2_FK/p2_cinematica_directa.py` | `ForwardKinematics.compute()` | T0_6 con numpy, tabla de error, grafica workspace |
| P3 | `P3_IK/p3_cinematica_inversa.py` | `InverseKinematics.solve_analytical()` | IK analitica + comparacion vs. API |
| P4 | `P4_COLISIONES/p4_evasion_colisiones.py` | `CollisionChecker.safe_move()` | Limites conservadores + FK height check + waypoint |
| P5 | `P5_CONTROL/control.py` | `RobotController.run_cycle()` | Ciclo pick&place, 5 ciclos, metricas |
| P6 | `P6_VISION/vision.py` | `ObjectDetector.detect_object()` | HSV, centroide, pixel->mm |
| P7 | `P7_PIPELINE/main.py` | `Pipeline.run()` | Maquina de estados E2E |

---

## Parametros DH del MyCobot 280

| Joint | a_i (mm) | d_i (mm) | alpha_i | theta_i | Rango |
|-------|-----------|-----------|---------|---------|-------|
| J1 Base | 0 | 131.56 | 90 deg | theta_1 | -168 a 168 |
| J2 Hombro | 110.4 | 0 | 0 deg | theta_2 | -135 a 90 |
| J3 Codo | 96 | 0 | 0 deg | theta_3 | -150 a 150 |
| J4 Muneca 1 | 0 | 66.39 | -90 deg | theta_4 | -145 a 145 |
| J5 Muneca 2 | 0 | 73.18 | 90 deg | theta_5 | -165 a 165 |
| J6 Gripper | 0 | 48.6 | 0 deg | theta_6 | -180 a 180 |

---

## Inicializacion del Robot (todos los roles)

```python
from pymycobot.mycobot import MyCobot
import time

mc = MyCobot('/dev/ttyUSB0', 1000000)
mc.power_on()
time.sleep(0.5)
assert mc.is_controller_connected(), "Error de conexion"

mc.send_angles([0, 0, 0, 0, 0, -45], 50)  # pose de reposo segura
time.sleep(3)
print("Posicion actual:", mc.get_coords())
```

---

## Como ejecutar cada problema

### P1 - Tabla DH y matrices
```bash
cd P1_DH
python p1_dh_representacion.py
```

### P2 - Cinematica Directa (con hardware)
```python
from pymycobot.mycobot import MyCobot
from P2_FK.p2_cinematica_directa import run_fk_verification
mc = MyCobot('/dev/ttyUSB0', 1000000)
mc.power_on()
run_fk_verification(mc=mc)
```

### P3 - Cinematica Inversa (con hardware)
```python
from pymycobot.mycobot import MyCobot
from P3_IK.p3_cinematica_inversa import compare_ik_solutions
mc = MyCobot('/dev/ttyUSB0', 1000000)
mc.power_on()
compare_ik_solutions(mc=mc)
```

### P4 - Evasion de Colisiones
```python
from pymycobot.mycobot import MyCobot
from P4_COLISIONES.p4_evasion_colisiones import validate_with_real_robot
mc = MyCobot('/dev/ttyUSB0', 1000000)
mc.power_on()
validate_with_real_robot(mc)
```

### P5 - 5 Ciclos consecutivos
```python
from pymycobot.mycobot import MyCobot
from P5_CONTROL.control import run_5_cycles
mc = MyCobot('/dev/ttyUSB0', 1000000)
run_5_cycles(mc, pick_x=150, pick_y=0)
```

### P6 - Deteccion en vivo
```python
from P6_VISION.vision import run_live_detection
run_live_detection(camera_id=0, target_color="red")
```

### P7 - Pipeline E2E completo
```python
from pymycobot.mycobot import MyCobot
from P7_PIPELINE.main import Pipeline
mc = MyCobot('/dev/ttyUSB0', 1000000)
pipeline = Pipeline(mc, camera_id=0, target_color='red', n_cycles=5)
pipeline.run()
```

---

## Arquitectura del Sistema

```
                    +------------------+
                    |   main.py (P7)   |
                    | State Machine    |
                    +--------+---------+
                             |
          +------------------+------------------+
          |                  |                  |
    +-----+------+    +------+------+    +------+------+
    | vision.py  |    |cinematica.py|    | control.py  |
    | (P6)       |    | (P3)        |    | (P5)        |
    | detect_obj |    | ik_solve()  |    | pick()      |
    | pixel->mm  |    | FK/IK/Coll  |    | place()     |
    +------------+    +-------------+    +-------------+
          |                  |                  |
    +-----+----+       +-----+----+       +-----+----+
    | OpenCV   |       |  numpy   |       | pymycobot |
    | Camera   |       |  math    |       | MyCobot   |
    +----------+       +----------+       +----------+
```

## Maquina de Estados (P7)

```
          +------+
          | IDLE |
          +--+---+
             |
          +--v------+         no detectado
          | DETECTAR+------------------------+
          +--+------+                        |
             |                               |
          +--v------+         sin solucion   |
          | CALC_IK +---+                    |
          +--+------+   |                    |
             |          |                    |
          +--v------+   |    fallo           |
          | AGARRAR +---+---+                |
          +--+------+       |                |
             |              |                |
          +--v--------+     |                |
          | DEPOSITAR |     v                v
          +--+--------+  +------+  --------+
             |           | ERROR+-->  IDLE  |
             |           +------+  ---------+
             |
             v (siguiente ciclo o fin)
```

---

## Fuentes del Codigo Base

Los codigos de este examen reutilizan y extienden los siguientes modulos del proyecto `src/`:

| Modulo src | Funcionalidad usada |
|-----------|---------------------|
| `jetcobot_utils/grasp_controller.py` | Poses, ciclo de agarre, gripper |
| `jetcobot_color_identify/identify_target.py` | Deteccion HSV, centroide, pixel->arm |
| `jetcobot_utils/color_recognition.py` | ColorRecognition, get_Sqaure |
| `jetcobot_advance/transform.py` | Conversion matrices/angulos |
| `jetcobot_utils/jetcobot_config.py` | HSV config, calibracion, perspectiva |

---

## Normas de Seguridad

1. Nunca dejar el robot operando sin supervision humana.
2. Siempre comenzar en `init_pose = [0, 0, 0, 0, 0, -45]`.
3. Mantener las manos fuera del espacio de trabajo durante la ejecucion.
4. Velocidad maxima durante pruebas iniciales: 30-50%.
5. Ctrl+C activa la parada de emergencia (vuelve a init_pose).
6. Verificar trayectoria visualmente en modo lento antes de velocidad normal.
