import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt("tendon_log.csv", delimiter=",", skiprows=1)
steps = data[:, 0]
u1    = data[:, 1]
u2    = data[:, 2]
f1    = data[:, 3]   # en mm (déplacement résiduel)
f2    = data[:, 4]

fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)

axes[0].set_title("Contrôle des tendons (inspiré Fig. 2B(i))")
axes[0].plot(steps, u1, 'r-', label='u1 (Tendon 1)')
axes[0].plot(steps, u2, 'g-', label='u2 (Tendon 2)')
axes[0].set_ylabel("Longueur normalisée")
axes[0].legend(); axes[0].grid(True)

axes[1].set_title("Résidu de déplacement des nœuds d'ancrage (proxy de force)")
axes[1].plot(steps, f1, 'r-', label='T1 résidu [mm]')
axes[1].plot(steps, f2, 'g--', label='T2 résidu [mm]')
axes[1].axhline(0, color='k', linewidth=0.5)
axes[1].set_ylabel("Déplacement résiduel [mm]")
axes[1].legend(); axes[1].grid(True)

axes[2].set_title("Repère morphologique : 4 états")
for xpos, label in zip(
    [0, int(len(steps)*0.2), int(len(steps)*0.6), len(steps)-1],
    ["Circulaire", "Rover", "Reconfiguration", "Sphère"]
):
    axes[2].axvline(steps[xpos], color='gray', linestyle=':')
    axes[2].text(steps[xpos]+1, 0.5, label, fontsize=8, rotation=90, color='gray')
axes[2].plot(steps, u1 - u2, 'b-', label='u1 - u2 (aspect ratio)')
axes[2].set_ylabel("Différence u1-u2"); axes[2].set_xlabel("Step")
axes[2].legend(); axes[2].grid(True)

plt.tight_layout()
plt.savefig("tendon_analysis.png", dpi=150)
plt.show()
print("tendon_analysis.png sauvegardé")