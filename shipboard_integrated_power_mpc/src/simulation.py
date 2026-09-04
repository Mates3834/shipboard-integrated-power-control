import numpy as np
from .model import ShipboardPowerModel
from .scenarios import load_profile
from .estimators import DiscreteKalmanFilter
from .controllers import PIDController, LQRController, MPCController
from .power_management import supervise_loads


def run(method="mpc", scenario="acceleration", duration=120.0, dt=0.1, seed=2):
    rng = np.random.default_rng(seed)
    model = ShipboardPowerModel(dt=dt)
    A, B, E, C = model.linear_matrices()
    kf = DiscreteKalmanFilter(A, B, C, dt)

    if method == "pid":
        ctrl = PIDController(dt=dt)
    elif method == "lqr":
        ctrl = LQRController(A, B)
    elif method == "mpc":
        ctrl = MPCController(model)
    else:
        raise ValueError("method must be pid, lqr, or mpc")

    x = np.array([0.5, 0.0, 0.0])
    u = 0.5
    out = {"t": [], "state": [], "estimate": [], "demand": [], "served": [], "u": []}

    for k in range(int(duration/dt)):
        t = k*dt
        demand = float(np.clip(load_profile(t, scenario), 0.0, 1.0))

        y = x + rng.normal(0.0, [0.02, 0.01, 0.02])
        xhat = kf.step([u], y)

        available = max(0.0, min(1.0, xhat[0]))
        _, served = supervise_loads(demand, available)
        control_target = demand if served >= demand - 1e-6 else served

        if method == "pid":
            u = ctrl(xhat[0], control_target)
        else:
            u = ctrl(xhat, control_target)

        x = model.step(x, [u], demand)

        out["t"].append(t)
        out["state"].append(x.copy())
        out["estimate"].append(xhat.copy())
        out["demand"].append(demand)
        out["served"].append(served)
        out["u"].append(u)

    for k in ("t","state","estimate","demand","served","u"):
        out[k] = np.asarray(out[k])
    return out
