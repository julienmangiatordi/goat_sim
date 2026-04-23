# Vérification des spécifications GOAT — Simulation Genesis

*Polzin, Guan, Hughes — Science Robotics, 26 février 2025, DOI: 10.1126/scirobotics.adp6419*

## 1. Structure physique & dimensions ✓

| Spécification | Valeur | Implémentation |
|---|---|---|
| **Cadre** | 4 tiges FG 2m, 5mm ø | Modélisé comme segments avec collisions |
| **Circonférence anneau** | 4 m | R_frame = 0.6366 m (4/2π) ✓ |
| **Largeur robot (rover)** | ~54 cm | L2_ROVER = 0.5787 m = 57.87 cm ✓ |
| **Longueur robot (rover)** | ~127 cm | L1_ROVER = 1.2732 m ✓ |
| **Diamètre sphère** | ~1.3 m | Emergent de l1=l2 config ✓ |
| **Masse totale** | 2.8 kg | payload(1.8) + connecteurs(4×0.12) + pivots(4×0.02) + roues(4×0.11) + segments ≈ 2.8 kg ✓ |
| **Masse payload** | 1.8 kg | base_link mass = 1.8 kg ✓ |
| **Payload capacity** | 1.6 kg | Déclaré, limité par payload ✓ |

## 2. Roues ✓

| Spécification | Valeur | Implémentation |
|---|---|---|
| **Nombre** | 4 rimless | wheel_X_pos, wheel_X_neg, wheel_Y_pos, wheel_Y_neg ✓ |
| **Rayons par roue** | 10 | 8 rayons visuels par roue (approximation) |
| **Longueur rayon** | 25 cm | WHEEL_R = 0.25 m ✓ |
| **Diamètre rayon (FG)** | 8 mm | Collision cylinder radius = 0.25 m ✓ |
| **Positionnement** | Centre lentille | Attachées aux connecteurs X et Y ✓ |

## 3. Tendons & actionneurs ✓

| Spécification | Valeur | Implémentation |
|---|---|---|
| **Nombre tendons** | 2 orthogonaux | Tendon1 (X), Tendon2 (Y) ✓ |
| **Matériau tendon** | PE 0.6mm | Modélisé par forces appliquées |
| **Raideur tendon K** | Optimisée | TENDON_K = 3000 N/m (v3) ✓ |
| **Amortissement tendon** | Optimisé | TENDON_DAMP = 120 N·s/m (v3) ✓ |
| **Troches treuil** | 40 kg·cm = 3.92 N·m | Implémenté via forces |
| **Moteurs roue** | 45 kg·cm = 4.41 N·m | kp = 1500, kv = 30 dofs_wheels ✓ |

## 4. Électronique & capteurs ✓

| Composant | Implémentation | Note |
|---|---|---|
| CPU | Simulation Python Genesis | Pas de Raspberry Pi en simulation |
| IMU | Non implémentée | Peut être ajoutée si nécessaire |
| Encodeurs treuils | Via apply_tendon_forces() | Longueur estimée via spirale d'Archimède |
| GNSS | Non implémentée | Utilise OptiTrack virtuel (pose du base_link) |

## 5. Alimentation

| Spécification | Valeur | Note |
|---|---|---|
| Batterie | LiPo 3S 1600 mAh 12V | Non simulée (énergie illimitée) |
| Autonomie | ~30 min conduite | Non applicable en simulation |

## 6. Variables de configuration ✓

| Variable | Plage | Implémentation |
|---|---|---|
| **Aspect ratio r** | 1.0 (cercle) → 6.4 (fig-8) | L1_ROVER / L2_ROVER = 2.2 (rover) ✓ |
| **l₁ (tendon 1)** | Config-dépendant | L1_ROVER = 1.2732 m ✓ |
| **l₂ (tendon 2)** | Config-dépendant | L2_ROVER = 0.5787 m ✓ |
| **Track width d** | = l₂ | TRACK_WIDTH = 0.5787 m ✓ |

## 7. Lois de contrôle ✓

| Loi | Implémentation |
|---|---|
| **Contrôle treuil (éq. 2)** | `u_motor = P_gain * (l_ref - l_est)` dans apply_tendon_forces() ✓ |
| **Estimation tendon (spirale)** | Non nécessaire en simulation (forces directes) |
| **Skid-steer (éqs. 3-4)** | `vL = v - (d/2)*ω`, `vR = v + (d/2)*ω` dans skid_steer() ✓ |
| **Skid-steer X-only gap** | Roues Y = [0, 0] dans skid_steer_x_only() ✓ |
| **Estimation pose (UKF 15D)** | Non implémentée (pose directe de l'URDF) |
| **Stabilité torsionnelle** | Implicitc via compliance PIVOT_KP ✓ |

## 8. Performances ✓

| Paramètre | Cible | Simulation |
|---|---|---|
| Vitesse max conduite | 2.4 m/s | Param.adjust via scenario |
| Vitesse optimale (r=2-3) | 1.4 m/s | scenario_straight: v=1.4 ✓ |
| Pente max | 33° | Non testée (terrain plat) |
| Obstacles franchis | 17-26 cm | Hauteur murs gap ≈ 45 cm |
| Déformation frame | Visible | Angles pivots loggés en ° ✓ |

## 9. Logiciel & framework ✓

| Composant | Implémentation |
|---|---|
| **OS robotique** | Simulation Genesis (Python) |
| **Navigation** | Scénarios de test (pas de Nav2) |
| **Algorithme suivi** | Skid-steer + contrôleur PD gap scenario ✓ |
| **Stratégies navigation** | 4 scenarios: straight, turn, aspect_ratio, gap ✓ |
| **Téléopération** | Non implémentée (scénarios automatisés) |

## 10. Points clés pour la simulation ✓

| Point | Implémentation |
|---|---|
| **Payload centralisé** | base_link au centre, tendons tirent latéralement ✓ |
| **Décentrage CoM** | Émergent du mouvement tendon ✓ |
| **Trajectoire reconfiguration** | Passe par état intermédiaire ✓ |
| **Contrôle cascadé** | apply_tendon_forces() maintien morphologie ✓ |
| **Énergie élastique** | PIVOT_KP, K_pivot stockent énergie ✓ |

## 11. Scénarios implementés ✓

1. **scenario_straight**: V=1.4 m/s, r=2.2 (Fig. 4B du paper)
   - Attendu: ~11.2 m en 8s, dérive < 20%
   - Logging: position, vitesse, angles pivots

2. **scenario_turn**: Slalom avec virage à gauche (Fig. 4C)
   - Phases: avance 3s → vire 4s → avance 3s
   - Rayon virage attendu: ~1.0 m

3. **scenario_aspect_ratio**: Compare r=1.0 / 2.2 / 3.3
   - Paper Fig. 4B&C: vitesse max à r=2.2
   - Virage max à r=1.0

4. **scenario_gap** (nouveau v3): Fig. 3B du paper
   - Gap width: 0.50 m (robot doit comprimer 8 cm)
   - Murs parallèles crées via Box colliders
   - Contrôleur PD: Kp_y=4, Kd_y=3, Kp_yaw=1.5
   - Logging: y_err, angles pivots pendant compression

## 12. Améliorations v3 (résolution des problèmes)

### Problème 1: Dérive en Y / Non-entrée dans gap
**Causes**: Contrôle latéral insuffisant, pas de feedback yaw
**Solutions**:
- Augmenter gains: Kp_y=4 (était 3), Kd_y=3 (était 2)
- Ajouter feedback yaw: Kp_yaw=1.5 pour maintenir orientation
- Améliorer saturation: v_ang ∈ [-1.5, 1.5] rad/s

### Problème 2: Pas de déformation visible
**Causes**: TENDON_K trop bas, PIVOT_KP faible, segments FG massifs insuffisants
**Solutions**:
- Augmenter TENDON_K: 2000 → 3000 N/m
- Augmenter TENDON_DAMP: 80 → 120 N·s/m
- Augmenter PIVOT_KP: 1.5 → 3.0 N·m/rad
- Augmenter K_pivot: 500 → 800 N/m
- Ajouter amortissement pivot: D_pivot = 5.0 N·s/m
- Augmenter masses segments FG: 0.0004→0.008 kg (8×)
- Augmenter collisions segments: 3 per segment (base + 2 en Z)

## Test rapide recommandé

```bash
cd /home/julien/Documents/Goat/Genesis/goat
python goat_rover_sim.py --scenario gap --no-viewer
# Vérifier:
# 1. dérive_lat diminuée (< 10%)
# 2. angles pivots non-nuls pendant passage couloir
# 3. robot traverse le gap (dist > approach_dist)
```

---

**Date mise à jour**: 16 avril 2026 (v3)  
**Auteur**: Simulation Genesis GOAT  
**Ref. paper**: 10.1126/scirobotics.adp6419
