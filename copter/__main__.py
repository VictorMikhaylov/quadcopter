import argparse

from copter.controller import ControlSystem
from copter.quadcopter import Quadcopter
from copter.simulator import Simulator

DT = 0.01  # шаг моделирования системы (например одна сотая секунды)
T_END = 10  # конечное время моделирования (например 20 сек)

# Установим начальное положение системы
START_POINT = [0.0, 0.0, 0.0]
START_THETA = [0.0, 0.0, 0.0]

parser = argparse.ArgumentParser(
    prog="Quadcopter simulator",
    description="Simulate copter flight",
    epilog="Happy New Year!",
)

if __name__ == "__main__":
    parser.add_argument(
        "-d", "--dimention", dest="dim", choices=["altitude", "pitch"], required=True
    )
    args = parser.parse_args()

    # Создадим объект контроллера и объект коптер
    control_system = ControlSystem()

    quadcopter = Quadcopter(
        control_system, start_point=START_POINT, start_theta=START_THETA
    )

    # Установим целевое положение для нашей системы
    point = [0.0, 0.0, 10.0]  # метры
    theta = [0.0, 0.52, 0.0]  # радианы

    quadcopter.goto(point, theta)

    # Создадим объект симулятора и передадим в него коптер
    sim = Simulator(T_END, DT, quadcopter)
    # запуск симулятора
    if args.dim == "altitude":
        sim.runAltitudeSimulation()
    elif args.dim == "pitch":
        sim.runPitchSimulation()

    sim.showPlots()  # построение графиков
