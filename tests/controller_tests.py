import logging

from copter.__main__ import DT
from copter.controller import (
    K_D,
    K_I,
    K_P,
    K_T_D,
    K_T_I,
    K_T_P,
    K_V_D,
    K_V_I,
    K_V_P,
    ControlSystem,
)

LOGGER = logging.getLogger(__name__)

# Установим целевое положение для нашей системы
POSITION_DEST = [0.0, 0.0, 10.0]  # метры
THETA_DEST = [0.0, 0.52, 0.0]  # радианы


def _init_cs():
    return ControlSystem(K_P, K_I, K_D, K_T_P, K_T_I, K_T_D, K_V_P, K_V_I, K_V_D)


def test_nonequal_altitude_step():
    start_altitude = 0.0
    control_system = _init_cs()
    control_system.desiredPosition = POSITION_DEST[2]
    u = control_system.compute_motor_velocity(start_altitude, DT)
    LOGGER.debug(u)
    assert u != [0.0, 0.0, 0.0, 0.0]


def test_equal_altitude_step():
    start_altitude = POSITION_DEST[2]
    control_system = _init_cs()
    control_system.desiredPosition = POSITION_DEST[2]
    u = control_system.compute_motor_velocity(start_altitude, DT)
    LOGGER.debug(u)
    assert u == [0.0, 0.0, 0.0, 0.0]


def test_nonequal_theta_step():
    start_theta = 0.0
    start_theta_velocity = 0.0
    control_system = _init_cs()
    control_system.desiredTheta = THETA_DEST[1]
    u = control_system.compute_motor_velocity_theta(
        start_theta, start_theta_velocity, DT
    )
    LOGGER.debug(u)
    assert u[0] != u[2]
    assert u[1] == u[3]


def test_equal_theta_step():
    start_theta = THETA_DEST[1]
    start_theta_velocity = 0.0
    control_system = _init_cs()
    control_system.desiredTheta = THETA_DEST[1]
    u = control_system.compute_motor_velocity_theta(
        start_theta, start_theta_velocity, DT
    )
    LOGGER.debug(u)
    assert u.tolist() == [0.0, 0.0, 0.0, 0.0]
