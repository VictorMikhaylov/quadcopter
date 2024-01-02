class Pid:
    def __init__(self, k_p, k_i, k_d):
        """

        :param k_p: пропорциональный коэффициент регулятора
        :type k_p: float
        :param k_i: интегральный коэффициент регулятора
        :type k_i: float
        :param k_d: дифференциальный коэффициент регулятора
        :type k_d: float

        """

        # объявим приватные поля класса
        self._k_p = k_p
        self._k_i = k_i
        self._k_d = k_d
        self._previous_error_cmd = 0.0
        self._integration__error_cmd = 0.0

    def updatedPid(self, cmd_des, cmd_current, dt):
        assert dt != 0.0
        _error_cmd = cmd_des - cmd_current
        self._integration__error_cmd += _error_cmd * dt
        _differentiation_error_cmd = (_error_cmd - self._previous_error_cmd) / dt
        self._previous_error_cmd = _error_cmd

        return (
            self._k_p * _error_cmd
            + self._k_i * self._integration__error_cmd
            + self._k_d * _differentiation_error_cmd
        )


class ThetaPid:
    def __init__(self, k_t_p, k_t_i, k_t_d, k_v_p, k_v_i, k_v_d):
        """

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

        """

        # объявим приватные поля класса
        self._k_t_p = k_t_p
        self._k_t_i = k_t_i
        self._k_t_d = k_t_d
        self._k_v_p = k_v_p
        self._k_v_i = k_v_i
        self._k_v_d = k_v_d
        self._previous_error_theta = 0.0
        self._previous_error_velocity = 0.0
        self._integration__error_theta = 0.0
        self._integration__error_velocity = 0.0

    def updatedPid(self, theta_des, theta_current, theta_velocity_current, dt):
        theta_velocity_des = self._calc_theta_velocity(theta_des, theta_current, dt)
        acc_des = self._calc_theta_acc(theta_velocity_des, theta_velocity_current, dt)
        return acc_des

    def _calc_theta_velocity(self, theta_des, theta_current, dt):
        assert dt != 0.0
        error_theta = theta_des - theta_current
        differentiation_error_theta = (error_theta - self._previous_error_theta) / dt
        self._integration__error_theta += error_theta * dt
        self._previous_error_theta = error_theta

        return (
            self._k_t_p * error_theta
            + self._k_t_i * self._integration__error_theta
            + self._k_t_d * differentiation_error_theta
        )

    def _calc_theta_acc(self, theta_velocity_des, theta_velocity_current, dt):
        assert dt != 0.0
        error_velocity = theta_velocity_des - theta_velocity_current
        differentiation_error_velocity = (
            error_velocity - self._previous_error_velocity
        ) / dt
        self._integration__error_velocity += error_velocity * dt
        self._previous_error_velocity = error_velocity

        return (
            self._k_v_p * error_velocity
            + self._k_v_i * self._integration__error_velocity
            + self._k_v_d * differentiation_error_velocity
        )
