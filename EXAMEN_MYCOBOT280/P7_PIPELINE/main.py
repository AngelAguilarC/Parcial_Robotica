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
