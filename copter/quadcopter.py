from copter.controller import ControlSystem
from copter.model import VehicleSimpleDynamic

# Масса ЛА
MASS = 0.006
# Коэффициент тяги двигателя ЛА
K_B = 3.9865e-08
# Момент инерции по оси тангажа
I_Y = 7.16914e-05
# Количество двигателей ЛА
ROTOR_COUNT = 4
# Плечо ротора
L = 0.17


class Quadcopter:
    def __init__(
        self,
        control_system: ControlSystem,
        start_point: [float] = [0.0, 0.0, 0.0],
        start_theta: [float] = [0.0, 0.0, 0.0],
    ):
        self._control_system = control_system
        self._math_model = VehicleSimpleDynamic(
            MASS,
            K_B,
            I_Y,
            ROTOR_COUNT,
            L,
            start_point=start_point,
            start_theta=start_theta,
        )

        self._current_position = start_point[2]
        self._current_theta = start_theta[1]  # тангаж

    @property
    def currentPosition(self):
        return self._current_position

    @property
    def currentTheta(self):
        return self._current_theta

    def goto(self, point: [float], theta: [float]):
        self._control_system.desiredPosition = point[2]
        self._control_system.desiredTheta = theta[1]

    def updatedState(self, dt):
        # рассчитываем новое управляющие воздействие
        # на основе текущей высоты(_current_altitude) ЛА
        u = self._control_system.compute_motor_velocity(self.currentPosition, dt)
        # Рассчитываем положение ЛА с учетом полученного
        # управляющего воздействия
        self._math_model.computeAltitude(u, dt)
        self._current_position = self._math_model.getAltitude

    def updatedTheta(self, dt):
        # рассчитываем новое управляющие воздействие
        # на основе текущего тангажа ЛА
        u = self._control_system.compute_motor_velocity_theta(
            self.currentTheta, self.getThetaVelocity, dt
        )
        # Рассчитываем положение ЛА с учетом полученного
        # управляющего воздействия
        self._math_model.computeTheta(u, dt)
        self._current_theta = self._math_model.getTheta

    @property
    def getPosition(self):
        return self.currentPosition

    @property
    def getVelocity(self):
        return self._math_model.getVelocity

    @property
    def getAcceleration(self):
        return self._math_model.getAcceleration

    @property
    def getTheta(self):
        return self.currentTheta

    @property
    def getThetaVelocity(self):
        return self._math_model.getThetaVel

    @property
    def getThetaAcceleration(self):
        return self._math_model.getThetaAcc
