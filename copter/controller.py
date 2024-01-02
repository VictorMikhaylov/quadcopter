import numpy as np

from copter.pid import Pid, ThetaPid

#  Объявим параметры для моделирования
K_P = 300  # коэффициент Пропорционального регулирования
K_I = 35  # коэффициент Интегрального регулирования
K_D = 180  # коэффициент Дифференциального регулирования

K_T_P = 50  # коэффициент Пропорционального регулирования углового положения
K_T_I = 3  # коэффициент Интегрального регулирования углового положения
K_T_D = 5  # коэффициент Дифференциального регулирования углового положения

K_V_P = 200  # коэффициент Пропорционального регулирования угловой скорости
K_V_I = 1  # коэффициент Интегрального регулирования угловой скорости
K_V_D = 1  # коэффициент Дифференциального регулирования угловой скорости

# Ограничение на угловую скорость двигателей рад/сек
ROTOR_SPEED_LIMIT = 1000
# Ограничение на угловое ускорение коптера
THETA_ACC_LIMIT = 6.0  # рад / сек
# Требуемый уровень тяги
T_CMD = 10  # установие как константу


class ControlSystem:
    def __init__(
        self,
        k_p=K_P,
        k_i=K_I,
        k_d=K_D,
        k_t_p=K_T_P,
        k_t_i=K_T_I,
        k_t_d=K_T_D,
        k_v_p=K_V_P,
        k_v_i=K_V_I,
        k_v_d=K_V_D,
        rotor_speed_limit=ROTOR_SPEED_LIMIT,
        theta_acc_limit=THETA_ACC_LIMIT,
    ):
        """

        :param k_p: коэффициент П регулятора
        :type k_p: float
        :param k_i: коэффициент И регулятора
        :type k_i: float
        :param k_d: коэффициент Д регулятора
        :type k_d: float
        :param k_t_p: пропорциональный коэффициент углового положения
        :type k_t_p: float
        :param k_t_i: интегральный коэффициент углового положения
        :type k_t_i: float
        :param k_t_d: дифференциальный коэффициент углового положения
        :type k_t_d: float
        :param k_v_p: пропорциональный коэффициент регулятора скорости
        :type k_v_p: float
        :param k_v_i: интегральный коэффициент регулятора скорости
        :type k_v_i: float
        :param k_v_d: дифференциальный коэффициент регулятора скорости
        :type k_v_d: float
        :param rotor_speed_limit: ограничение по управляющему воздействию
        :type rotor_speed_limit: int
        :param theta_acc_limit: ограничение по управляющему воздействию
        :type theta_acc_limit: float

        """
        self._pid = Pid(k_p, k_i, k_d)
        self._tpid = ThetaPid(k_t_p, k_t_i, k_t_d, k_v_p, k_v_i, k_v_d)
        self._rotor_speed_limit = rotor_speed_limit
        self._theta_acc_limit = theta_acc_limit
        self._position_des = 0.0
        self._theta_des = 0.0

    @property
    def desiredPosition(self):
        return self._position_des

    @desiredPosition.setter
    def desiredPosition(self, position_des):
        """

        :param position_des: целевое положение ЛА (высота)
        :type position_des: float

        """
        self._position_des = position_des

    @property
    def desiredTheta(self):
        return self._theta_des

    @desiredTheta.setter
    def desiredTheta(self, theta_des):
        """

        :param pitch_des: целевое положение ЛА (тангаж)
        :type position_des: float

        """
        self._theta_des = theta_des

    def compute_motor_velocity(self, position_current, dt):
        """

        :param currentPosition: текущее положение ЛА
        :type currentPosition: float
        :param dt: шаг моделирования
        :type dt: float

        """
        u = self._pid.updatedPid(self._position_des, position_current, dt)

        u = self._mixer(u)

        return u

    def _mixer(self, cmd):
        # Вызовем звено насыщения для ограничения максимального управляющего воздействия
        cmd = np.clip(cmd, -self._rotor_speed_limit, self._rotor_speed_limit)

        # Этот метод обеспечивает перевод команд от системы управления.
        # в желаемую угловую скорость для каждого двигателя.
        return [cmd, cmd, cmd, cmd]

    def compute_motor_velocity_theta(self, theta_current, theta_velocity_current, dt):
        """

        :param currentPosition: текущее положение ЛА
        :type currentPosition: float
        :param dt: шаг моделирования
        :type dt: float

        """
        u = self._tpid.updatedPid(
            self._theta_des, theta_current, theta_velocity_current, dt
        )

        u = self._mixer_theta(u)

        return u

    def _mixer_theta(self, cmd):
        # Этот метод обеспечивает перевод команд от системы управления.
        # в желаемую угловую скорость для каждого двигателя.
        u = [T_CMD - cmd, 0.0, T_CMD + cmd, 0.0]
        return np.clip(u, 0, self._rotor_speed_limit)
