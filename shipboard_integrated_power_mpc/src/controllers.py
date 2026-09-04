import numpy as np
from scipy.linalg import solve_continuous_are
from scipy.optimize import minimize


class PIDController:
    def __init__(self, kp=1.8, ki=0.15, kd=0.2, dt=0.1):
        self.kp, self.ki, self.kd, self.dt = kp, ki, kd, dt
        self.i = 0.0
        self.prev = 0.0

    def __call__(self, power_estimate, demand):
        e = demand - power_estimate
        self.i += e*self.dt
        d = (e - self.prev)/self.dt
        self.prev = e
        return float(np.clip(demand + self.kp*e + self.ki*self.i + self.kd*d, 0.0, 1.0))


class LQRController:
    def __init__(self, A, B):
        Q = np.diag([12.0, 4.0, 3.0])
        R = np.array([[0.8]])
        P = solve_continuous_are(A, B, Q, R)
        self.K = np.linalg.solve(R, B.T @ P)

    def __call__(self, xhat, demand):
        # Track equilibrium power=demand, thermal/shaft deviations=0.
        xref = np.array([demand, 0.0, 0.0])
        u = demand - float(self.K @ (xhat - xref))
        return float(np.clip(u, 0.0, 1.0))


class MPCController:
    def __init__(self, model, horizon=12):
        self.model = model
        self.H = horizon
        self.prev_u = 0.5

    def __call__(self, xhat, demand):
        H = self.H

        def objective(U):
            x = np.asarray(xhat, dtype=float).copy()
            J = 0.0
            prev = self.prev_u
            for uk in U:
                x = self.model.step(x, [uk], demand)
                power_error = x[0] - demand
                J += (15.0*power_error**2
                      + 3.0*x[1]**2
                      + 1.5*x[2]**2
                      + 0.8*(uk-prev)**2)
                prev = uk
            return J

        x0 = np.full(H, self.prev_u)
        bounds = [(0.0, 1.0)] * H
        res = minimize(objective, x0, bounds=bounds, method="L-BFGS-B",
                       options={"maxiter": 35})
        u = float(res.x[0])
        self.prev_u = u
        return u
