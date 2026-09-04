import numpy as np


def metrics(result):
    state = result["state"]
    demand = result["demand"]
    u = result["u"]

    error = state[:,0] - demand
    du = np.diff(u, prepend=u[0])

    return {
        "power_rmse": float(np.sqrt(np.mean(error**2))),
        "peak_power_error": float(np.max(np.abs(error))),
        "peak_thermal_deviation": float(np.max(np.abs(state[:,1]))),
        "control_effort": float(np.sum(u**2)),
        "control_rate_effort": float(np.sum(du**2)),
        "constraint_violations": int(np.sum((u < -1e-9) | (u > 1.0+1e-9))),
    }
