import numpy as np


def load_profile(t, scenario="cruise"):
    """
    Returns normalized total ship demand.
    All values are generic and dimensionless.
    """
    if scenario == "cruise":
        return 0.55
    if scenario == "acceleration":
        return 0.45 if t < 30 else 0.75
    if scenario == "heavy_sea":
        return 0.60 + 0.08*np.sin(0.18*t) + 0.03*np.sin(0.61*t)
    if scenario == "berthing":
        if t < 25:
            return 0.35
        if t < 55:
            return 0.70
        return 0.30
    if scenario == "load_shed_event":
        return 0.70 if t < 50 else 0.40
    return 0.55


def demand_components(total):
    """
    Generic priority decomposition:
    critical > propulsion > navigation > auxiliary > hotel.
    """
    fractions = {
        "critical": 0.18,
        "propulsion": 0.50,
        "navigation": 0.08,
        "auxiliary": 0.14,
        "hotel": 0.10,
    }
    return {k: total*v for k, v in fractions.items()}
