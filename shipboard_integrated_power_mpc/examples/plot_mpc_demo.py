import matplotlib.pyplot as plt
from src.simulation import run

r = run(method="mpc", scenario="heavy_sea")

plt.figure()
plt.plot(r["t"], r["demand"], label="Demand")
plt.plot(r["t"], r["state"][:,0], label="Generated power")
plt.plot(r["t"], r["estimate"][:,0], label="Estimated power", alpha=0.8)
plt.xlabel("Time [s]")
plt.ylabel("Normalized power")
plt.title("MPC Shipboard Power Tracking")
plt.grid(True)
plt.legend()
plt.show()
