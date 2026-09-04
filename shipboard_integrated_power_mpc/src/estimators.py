import numpy as np


class DiscreteKalmanFilter:
    def __init__(self, A, B, C, dt=0.1):
        # First-order discretization is sufficient for this educational model.
        n = A.shape[0]
        self.Ad = np.eye(n) + A*dt
        self.Bd = B*dt
        self.C = C
        self.Q = np.eye(n)*1e-5
        self.R = np.diag([2e-3, 5e-4, 2e-3])
        self.P = np.eye(n)*0.1
        self.x = np.zeros(n)

    def step(self, u, y):
        u = np.asarray(u).reshape(-1)
        self.x = self.Ad @ self.x + self.Bd.reshape(len(self.x)) * u[0]
        self.P = self.Ad @ self.P @ self.Ad.T + self.Q

        S = self.C @ self.P @ self.C.T + self.R
        K = self.P @ self.C.T @ np.linalg.inv(S)
        innovation = np.asarray(y) - self.C @ self.x
        self.x = self.x + K @ innovation
        self.P = (np.eye(len(self.x)) - K @ self.C) @ self.P
        return self.x.copy()
