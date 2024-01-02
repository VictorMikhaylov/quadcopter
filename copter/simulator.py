import numpy as np
from matplotlib import pyplot as plt

from copter.quadcopter import Quadcopter

AXIS_ALTITUDE = ["position", "velocity", "acceleration"]
AXIS_PITCH = ["pitch", "pitch velocity", "pitch acceleration"]


class Simulator:
    def __init__(self, Tend: float, dt: float, quadcopter: Quadcopter):
        """

        :param Tend: конечное время моделирования
        :type Tend: float
        :param dt: шаг моделирования
        :type dt: float
        :param quadcopter: объект квадрокоптера
        :type quadcopter: Quadcopter

        """
        self.dt = dt
        self.T_end = Tend
        self.quadcopter = quadcopter
        self.accList = []
        self.velList = []
        self.posList = []
        self.timeList = []
        self._axis_title = AXIS_ALTITUDE

    def runAltitudeSimulation(self):
        """

        метод запускает моделирование системы от 0 до конечного времени Tend
        с шагом dt

        """
        self._axis_title = AXIS_ALTITUDE
        # Задаем 0 время и начинаем рассчет до тех пор пока
        # время не достигнет конечного значения T_end
        time = 0
        for time in np.arange(0, self.T_end, self.dt):
            self.quadcopter.updatedState(self.dt)
            # Записываем полученные значения в списки
            # для дальнейшего построения графиков
            self.posList.append(self.quadcopter.currentPosition)
            self.velList.append(self.quadcopter.getVelocity)
            self.accList.append(self.quadcopter.getAcceleration)
            self.timeList.append(time)

            # увеличиваем время на dt, то есть на шаг моделирования
            time += self.dt

    def runPitchSimulation(self):
        """

        метод запускает моделирование системы от 0 до конечного времени Tend
        с шагом dt

        """
        self._axis_title = AXIS_PITCH
        # Задаем 0 время и начинаем рассчет до тех пор пока
        # время не достигнет конечного значения T_end
        time = 0
        for time in np.arange(0, self.T_end, self.dt):
            self.quadcopter.updatedTheta(self.dt)
            # Записываем полученные значения в списки
            # для дальнейшего построения графиков
            self.posList.append(self.quadcopter.currentTheta)
            self.velList.append(self.quadcopter.getThetaVelocity)
            self.accList.append(self.quadcopter.getThetaAcceleration)
            self.timeList.append(time)

            # увеличиваем время на dt, то есть на шаг моделирования
            time += self.dt

    def showPlots(self):
        """
        метод строит графики на основе измерений полученных в
        ходе моделирования системы

        """
        f = plt.figure(constrained_layout=True)
        gs = f.add_gridspec(3, 5)
        ax1 = f.add_subplot(gs[0, :-1])
        ax1.plot(self.timeList, self.posList)
        ax1.grid()
        ax1.set_title(self._axis_title[0])

        ax2 = f.add_subplot(gs[1, :-1])
        ax2.plot(self.timeList, self.velList, "g")
        ax2.grid()
        ax2.set_title(self._axis_title[1])

        ax3 = f.add_subplot(gs[2, :-1])
        ax3.plot(self.timeList, self.accList, "r")
        ax3.grid()
        ax3.set_title(self._axis_title[2])

        plt.show()
