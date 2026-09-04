from dataclasses import dataclass
import numpy as np


@dataclass
class PlantParameters:
    tau_power: float = 8.0
    thermal_capacity: float = 60.0
    heat_loss: float = 0.12
    tau_shaft: float = 3.0


class ShipboardPowerModel:
    """
    Generic reduced-order model.

    States:
      x[0] = normalized generated power
      x[1] = normalized thermal deviation
      x[2] = normalized shaft-speed deviation

    Input:
      u[0] = normalized power command

    Disturbance:
      d[0] = normalized ship power demand
    """

    def __init__(self, p=PlantParameters(), dt=0.1):
        self.p = p
        self.dt = dt

    def derivative(self, x, u, demand):
        p = self.p
        power = x[0]
        temp = x[1]
        shaft = x[2]
        cmd = float(np.asarray(u).reshape(-1)[0])

        dp = (cmd - power) / p.tau_power
        dT = (power - demand - p.heat_loss * temp) / p.thermal_capacity
        domega = (power - demand - shaft) / p.tau_shaft
        return np.array([dp, dT, domega], dtype=float)

    def step(self, x, u, demand):
        dt = self.dt
        f = lambda s: self.derivative(s, u, demand)
        k1 = f(x)
        k2 = f(x + 0.5 * dt * k1)
        k3 = f(x + 0.5 * dt * k2)
        k4 = f(x + dt * k3)
        return x + dt * (k1 + 2*k2 + 2*k3 + k4) / 6.0

    def linear_matrices(self):
        p = self.p
        A = np.array([
            [-1.0/p.tau_power, 0.0, 0.0],
            [1.0/p.thermal_capacity, -p.heat_loss/p.thermal_capacity, 0.0],
            [1.0/p.tau_shaft, 0.0, -1.0/p.tau_shaft],
        ])
        B = np.array([[1.0/p.tau_power], [0.0], [0.0]])
        E = np.array([[0.0], [-1.0/p.thermal_capacity], [-1.0/p.tau_shaft]])
        C = np.eye(3)
        return A, B, E, C
