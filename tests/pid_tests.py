import logging

from copter.__main__ import DT
from copter.controller import K_D, K_I, K_P, K_T_D, K_T_I, K_T_P, K_V_D, K_V_I, K_V_P
from copter.pid import Pid, ThetaPid

LOGGER = logging.getLogger(__name__)

#  Объявим параметры для моделирования
POSITION_DEST = 10.0  # метры

THETA_DEST = 0.52  # радианы


def test_pid_1():
    position_start = 0.0
    pid = Pid(K_P, K_I, K_D)
    u = pid.updatedPid(POSITION_DEST, position_start, DT)
    LOGGER.debug(u)
    assert u > 0.0


def test_pid_2():
    position_start = POSITION_DEST
    pid = Pid(K_P, K_I, K_D)
    u = pid.updatedPid(POSITION_DEST, position_start, DT)
    LOGGER.debug(u)
    assert u == 0.0


def test_tpid_1():
    theta_start = 0.0
    theta_velocity_start = 0.0
    tpid = ThetaPid(K_T_P, K_T_I, K_T_D, K_V_P, K_V_I, K_V_D)
    u = tpid.updatedPid(THETA_DEST, theta_start, theta_velocity_start, DT)
    LOGGER.debug(u)
    assert u > 0.0


def test_tpid_2():
    theta_start = THETA_DEST
    theta_velocity_start = 0.0
    tpid = ThetaPid(K_T_P, K_T_I, K_T_D, K_V_P, K_V_I, K_V_D)
    u = tpid.updatedPid(THETA_DEST, theta_start, theta_velocity_start, DT)
    LOGGER.debug(u)
    assert u == 0.0
