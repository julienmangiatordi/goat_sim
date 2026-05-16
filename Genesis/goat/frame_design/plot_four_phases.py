#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

LOG_FILE = "four_phases_log.csv"

PHASE_COLORS = {
    "Circulaire":      "#ddeeff",
    "Rover":           "#ddffdd",
    "Reconfiguration": "#ffe9c6",
    "Sphere":          "#ffd6d6",
}

def add_phase_bands(ax, df, y_min, y_max):
    for ph in df["phase"].unique():
        sub = df[df["phase"] == ph]
        ax.axvspan(sub["step"].min(), sub["step"].max(),
                   color=PHASE_COLORS.get(ph, "#f0f0f0"), alpha=0.18)

def main():
    df = pd.read_csv(LOG_FILE)

    # Vérification du format : nouveau CSV (fix1) ou ancien
    new_format = "l1_m" in df.columns

    fig, axes = plt.subplots(4 if new_format else 3, 1,
                             figsize=(11, 10 if new_format else 8),
                             sharex=True)
    fig.suptitle("GOAT — Simulation 4 phases (défaut 1 corrigé)" if new_format
                 else "GOAT — Simulation 4 phases",
                 fontsize=14, fontweight="bold", y=1.01)

    steps = df["step"]

    # ── Subplot 1 : gaps de tendon (l1, l2) en mètres ──────────────
    ax = axes[0]
    add_phase_bands(ax, df, 0, 1)
    if new_format:
        ax.plot(steps, df["l1_m"], label="l₁ — Tendon 1 (m)", color="#e07b00", lw=2)
        ax.plot(steps, df["l2_m"], label="l₂ — Tendon 2 (m)", color="#1a7a3f", lw=2)
        ax.set_ylabel("Gap tendon (m)")
        ax.set_title("Longueurs de gap des tendons")
        ax.axhline(1.65, color="gray", lw=0.8, ls=":", alpha=0.6, label="L_MAX = 1.65 m")
    else:
        ax.plot(steps, df["u1"], label="u1 (Tendon 1)", color="#e07b00", lw=2)
        ax.plot(steps, df["u2"], label="u2 (Tendon 2)", color="#1a7a3f", lw=2)
        ax.set_ylabel("Commande u")
        ax.set_title("Contrôle des tendons (u1, u2)")
    ax.legend(loc="upper right", fontsize=9)
    ax.set_ylim(bottom=-0.05)

    # ── Subplot 2 : aspect ratio ────────────────────────────────────
    ax = axes[1]
    add_phase_bands(ax, df, 0, 5)
    if new_format:
        ax.plot(steps, df["aspect"], color="#c0392b", lw=2, label="aspect = l₂/l₁")
        ax.axhline(1.0, color="gray", lw=0.8, ls="--", alpha=0.7, label="aspect=1 (cercle)")
        ax.set_ylabel("Aspect ratio")
        ax.set_title("Ratio d'aspect du frame (l₂ / l₁)")
        ax.legend(loc="upper right", fontsize=9)
        ax.set_ylim(0, df["aspect"].max() * 1.15)
    else:
        ax.plot(steps, df["u1_minus_u2"], color="#c0392b", lw=2, label="u1 − u2")
        ax.axhline(0.0, color="gray", lw=0.8, ls="--", alpha=0.7)
        ax.set_ylabel("u1 − u2")
        ax.set_title("Proxy de morphologie (aspect ratio)")
        ax.legend(loc="upper right", fontsize=9)

    # ── Subplot 3 : radius_scale et z_scale (nouveau format seulement) ──
    if new_format:
        ax = axes[2]
        add_phase_bands(ax, df, 0, 1.1)
        ax.plot(steps, df["radius_scale"], color="#2980b9", lw=2, label="radius_scale")
        ax.plot(steps, df["z_scale"],      color="#8e44ad", lw=2, label="z_scale")
        ax.axhline(1.0, color="gray", lw=0.8, ls=":", alpha=0.5)
        ax.set_ylabel("Facteur d'échelle")
        ax.set_title("Compaction radiale et hors-plan (Z)")
        ax.legend(loc="upper right", fontsize=9)
        ax.set_ylim(0.3, 1.15)

    # ── Subplot final : timeline des phases ────────────────────────
    ax = axes[-1]
    for ph in df["phase"].unique():
        sub = df[df["phase"] == ph]
        step_min = sub["step"].min()
        step_max = sub["step"].max()
        ax.axvspan(step_min, step_max,
                   color=PHASE_COLORS.get(ph, "#f0f0f0"), alpha=0.7)
        ax.text(0.5*(step_min + step_max), 0.5, ph,
                ha="center", va="center", fontsize=11, fontweight="bold")

    # Ajouter une ligne verticale aux frontières de phases
    phase_bounds = df.groupby("phase")["step"].min().sort_values()
    for s in phase_bounds.values[1:]:
        for a in axes:
            a.axvline(s, color="#555", lw=0.8, ls="--", alpha=0.4)

    ax.set_yticks([])
    ax.set_ylabel("Phase")
    ax.set_xlabel("Step de simulation")
    ax.set_xlim(steps.min(), steps.max())
    ax.set_title("Timeline des 4 phases")

    # Légende commune des phases
    patches = [mpatches.Patch(facecolor=c, alpha=0.7, label=ph)
               for ph, c in PHASE_COLORS.items()]
    fig.legend(handles=patches, loc="lower center", ncol=4,
               fontsize=9, bbox_to_anchor=(0.5, -0.02))

    plt.tight_layout()
    plt.savefig("four_phases_plots.png", dpi=200, bbox_inches="tight")
    print("Figure sauvegardée → four_phases_plots.png")


if __name__ == "__main__":
    main()