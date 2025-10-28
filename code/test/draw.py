import pandas as pd
import matplotlib.pyplot as plt
from scipy.stats import linregress

# === CSV-Datei einlesen ===
# Ersetze den Dateinamen ggf. durch deinen eigenen
filename = "no_drift_comp_260.csv"

# CSV laden (erste Zeile: Time_ms,Angle_deg)
data = pd.read_csv(filename)

# Umrechnung in Sekunden
data["Time_s"] = data["Time_ms"] / 1000.0

# === Plot ===
plt.figure()
plt.plot(data["Time_s"], data["Angle_deg"], label="Integrierter Winkel (ohne Driftkorrektur)")

# === Regression (optional) ===
slope, intercept, r_value, p_value, std_err = linregress(data["Time_s"], data["Angle_deg"])
drift_line = intercept + slope * data["Time_s"]
plt.plot(data["Time_s"], drift_line, linestyle="--", label=f"Linearer Drift: {slope:.4f} °/s")

# === Plot-Anpassung ===
plt.xlabel("Zeit [s]")
plt.ylabel("Winkel [°]")
plt.title("Drift-Analyse Gyroskop (MPU6050)")
plt.legend()
plt.grid(True)
plt.tight_layout()

plt.savefig("gyro_drift_plot_260.png")
print("Plot gespeichert als gyro_drift_plot.png")

