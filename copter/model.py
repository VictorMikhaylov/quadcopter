import numpy as np

# Величина ускорения свободного падения
G = 9.81


class VehicleSimpleDynamic:
    def __init__(
        self,
        mass: float,
        k_b: float,
        i_y: float,
        rotor_count: int,
        l: float,
        acc_init=0.0,
        vel_init=0.0,
        start_point: [float] = [0.0, 0.0, 0.0],
        acc_theta_init=0.0,
        vel_theta_init=0.0,
        start_theta: [float] = [0.0, 0.0, 0.0],
    ):
        """

        :param mass: масса аппарата
        :type mass: float
        :param k_b: коэффициент тяги двигателя
        :type k_b: float
        :param i_y: момент инерции
        :type i_y: float
        :param rotor_count: количество двигателей в системе
        :type rotor_count: int
        :param l: расстояние между двигателями
        :type l: float
        :param acc_init: начальное значение ускорения ЛА
        :type acc_init: float
        :param vel_init: начальное значение скорости ЛА
        :type vel_init: float
        :param altitude_init: начальное значение положения ЛА
        :type altitude_init: [float]
        :param vel_theta_init: начальное значение углового ускорения ЛА
        :type vel_theta_init: float
        :param vel_theta_init: начальное значение угловой скорости ЛА
        :type vel_theta_init: float
        :param start_theta: начальное значение углов наклона ЛА
        :type start_theta: [float]

        """

        self._mass = mass
        self._k_b = k_b
        self._i_y = i_y
        self._rotor_count = rotor_count
        self._l = l
        self._acceleration = acc_init
        self._velocity = vel_init
        self._altitude = start_point[2]
        self._theta_acc = acc_theta_init
        self._theta_velocity = vel_theta_init
        self._pitch = start_theta[1]

    def _rightParts(self, rotors_angular_vel):
        """

        :param rotorsAngularVel: угловая скорость двигателей
        :type rotorsAngularVel: float

        """

        # Проверяем что количество двигателей совпадает с размерностью команды
        if len(rotors_angular_vel) != self._rotor_count:
            raise ValueError("Incorrect number of motors")

        # Для всех двигателей рассчитаем суммарную тягу
        rotors_angular_vel_sum = np.sum(np.square(rotors_angular_vel))

        # Вычисляем ускорение нашей системы.
        # Получаем силу тяги создаваемую всеми двигателями
        # согласно нашей математической модели Fтяги = Kb * omega^2
        # После для получения текущего ускорения разделим полученное значение
        # на массу и вычтем ускорение свободного падения действующие на аппарат
        self._acceleration = (self._k_b * rotors_angular_vel_sum) / self._mass - G

    def computeAltitude(self, u, dt: float):
        """

        :param u: управляющие воздействие
        :type u: [float]
        :param dt: шаг моделирования
        :type dt: float

        """
        # Для определения положения вызываем метод для правых частей(то есть наших приращений от перемещения)
        # В данном случае приращением выступает ускорение нашей системы.
        # Для следования заданному целевому значению высоты передаем в метод наше управляющие воздействие u,
        # которое характеризует необходимую угловую скорость двигателей.
        self._rightParts(u)
        # Далее вызываем метод интегрирования
        # Интегрируем полученное при помощи функции правых частей ускорение и получаем скорость,
        # после интегрируем скорость и получаем положение.
        self._integrate(dt)

    def _thetaParts(self, u):
        """

        :param u: управляющие воздействие
        :type u: [float]

        """

        # Проверяем что количество двигателей совпадает с размерностью команды
        if len(u) != self._rotor_count:
            raise ValueError("Incorrect number of motors")

        m_y = self._k_b * self._l * (u[2] ** 2 - u[0] ** 2) / 2

        self._theta_acc = m_y / self._i_y

    def computeTheta(self, u, dt: float):
        """

        :param u: управляющие воздействие
        :type u: [float]
        :param dt: шаг моделирования
        :type dt: float

        """
        self._thetaParts(u)
        # Далее вызываем метод интегрирования
        # Интегрируем полученное при помощи функции правых частей ускорение и получаем скорость,
        # после интегрируем скорость и получаем положение.
        self._integrate(dt)

    def _integrate(self, dt: float):
        """

        :param dt: шаг моделирования
        :type dt: float

        """
        # интегрируем ускорение методом эйлера
        self._velocity += self._acceleration * dt
        # Полученную скорость интегрируем для определения местоположения
        self._altitude += self._velocity * dt

        self._theta_velocity += self._theta_acc * dt
        self._pitch += self._theta_velocity * dt

    @property
    def getAltitude(self):
        """

        :return: положение ЛА
        :rtype: float

        """
        return self._altitude

    @property
    def getVelocity(self):
        """

        :return: скорость ЛА
        :rtype: float

        """
        return self._velocity

    @property
    def getAcceleration(self):
        """

        :return: ускорение
        :rtype: float

        """
        return self._acceleration

    @property
    def getTheta(self):
        """

        :return: тангаж ЛА
        :rtype: float

        """
        return self._pitch

    @property
    def getThetaVel(self):
        """

        :return: угловая скорость ЛА
        :rtype: float

        """
        return self._theta_velocity

    @property
    def getThetaAcc(self):
        """

        :return: угловое ускорение ЛА
        :rtype: float

        """
        return self._theta_acc
