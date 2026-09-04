import numpy as np
from src.model import ShipboardPowerModel, PlantParameters
from src.evaluation import metrics
from src.simulation import run

# Lightweight robustness example over sensor-noise realizations.
records=[]
for seed in range(50):
    records.append(metrics(run(method="mpc", scenario="heavy_sea", seed=seed)))

for key in records[0]:
    vals=np.array([r[key] for r in records],dtype=float)
    print(f"{key}: mean={vals.mean():.4g}, std={vals.std():.4g}")
