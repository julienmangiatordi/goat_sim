# 🚀 QUICK START - GOAT Lens Frame Genesis Simulation

## En 2 minutes chrono ⏱️

### 1. Générer le mesh OBJ

```bash
cd /home/julien/Documents/Goat/Genesis/goat/frame_design
python generate_lens_frame_genesis.py
```

✓ Crée: `goat_lens_genesis.obj` avec 2112 vertices

### 2. Lancer la simulation simple

**Option A: Version simple (recommandé pour démarrer)**
```bash
python frame_scratch_simple.py
```
- Utilise les ring_A.obj, ring_B.obj existants
- ~10s pour initialiser + démarrer
- FEM Elastic stable

**Option B: Version complète avec mesh généré**
```bash
python frame_scratch.py
```
- Utilise le mesh généré (goat_lens_genesis.obj)
- ~30s pour tetrahedralization
- Plus complexe mais un seul maillage

**Option C: Version MPM (très robuste)**
```bash
python frame_scratch_mpm.py
```
- Résiste mieux aux géométries complexes
- ⏳ 2-5 min d'initialisation (sampling particules)
- Meilleur pour structure complexe

### 3. Observer la simulation

La fenêtre Genesis s'ouvre avec:
- **Bleu clair**: Anneaux du frame
- **Gris**: Obstacles (murs du scénario "gap")
- **Déformation**: Observation du frame se compresser dans le couloir

---

## 📦 Fichiers Expliqués

| Fichier | But | Démarrage | Temps |
|---------|-----|-----------|-------|
| `generate_lens_frame_genesis.py` | Génère mesh OBJ procéduralement | `python ...` | 2s |
| `frame_scratch_simple.py` | ⭐ Simple + rapide + stable | `python ...` | 10s |
| `frame_scratch.py` | Complet avec mesh généré | `python ...` | 30s |
| `frame_scratch_mpm.py` | MPM (le plus robuste) | `python ...` | 5min |
| `README_FRAME_GENESIS.md` | Documentation détaillée | - | - |
| `README_IMPLEMENTATION.md` | Architecture complète | - | - |

---

## 🎯 Paramètres à Ajuster

### Si simulation trop lente

**Dans `frame_scratch.py` ou `.  _simple.py`:**
```python
# Réduire discrétisation mesh
python generate_lens_frame_genesis.py --points 40 --sections 6

# Ou augmenter substeps
substeps=30  → substeps=15
```

### Si frame trop rigide (pas de déformation)

```python
# Diminuer E (Young's modulus)
E=5e6  → E=1e6  (plus déformable)
```

### Si frame trop mou (se déforme trop)

```python
# Augmenter E
E=5e6  → E=1e7  (plus rigide)
```

---

## 🔧 Architecture Rapide

```
GOAT Simulation Pipeline
│
├─ [1] generate_lens_frame_genesis.py → goat_lens_genesis.obj
│
├─ [2] frame_scratch_simple.py ✅ (Démarrer ici!)
│      └─ charge ring_A.obj + ring_B.obj + struts
│         pour éviter tetrahedralization complexe
│
├─ [3] frame_scratch.py
│      └─ utilise goat_lens_genesis.obj
│         (maillage complet unifié)
│
└─ [4] frame_scratch_mpm.py
       └─ MPM Solver (Ultra-robuste)
```

---

## 🐛 Troubleshooting

| Problème | Solution |
|----------|----------|
| `FileNotFoundError: ring_A.obj` | Vérifier d'être dans: `/home/julien/Documents/Goat/Genesis/goat/frame_design/` |
| Simulation très lente (1 fps) | Réduire `--points` en générant le mesh |
| Genesis crash | Essayer `frame_scratch_simple.py` à la place |
| Viewer ne s'affiche pas | Ajouter: `--no-viewer` et utiliser recording |
| Import trimesh error | `pip install trimesh` |

---

## 📊 Résultats Attendus

**frame_scratch_simple.py:**
```
Initializing Genesis...
🚀 Genesis initialized. version: 0.4.6

Creating scene with FEM Elastic solver...
Adding ground plane...
Setting up fiberglass material...
Adding frame components...
Adding obstacles (gap scenario)...
Building scene...

=======================================================================
SIMULATION RUNNING
=======================================================================
Frame material: Fiberglass (E=5e6 Pa)
Corridor width: 0.45 m (frame Y-width: 0.579 m)
Observer the frame compress through the gap
Press Ctrl+C to stop

Step  1000 / 10000
Step  2000 / 10000
...
```

Et une fenêtre 3D s'ouvre montrant:
- 2 anneaux croisés en bleu
- 2 murs gris de chaque côté
- Frame qui se déforme pour passer dans le couloir

---

## 🎓 Concepts Clés

### Deformable Beams = FEM + MPM

**Ce projet démontre:**
1. **Géométrie complexe**: 2 anneaux croisés formant une "lens"
2. **Physics réaliste**: Fiberglass (E≈5-40 GPa)
3. **Collision**: Contact avec obstacles (murs gap scenario)
4. **Déformation**: Morphing du frame pour adaptation morphologique
5. **Multi-solver**: FEM pour précision, MPM pour robustesse

### Paper Reference

- **Figure 2.B**: Lens structure
- **Paper**: GOAT (Gait Optimization And Topology)
- **Concepts**: Morphing compliant frame en câbles tendus

---

## 💡 Prochaines Étapes

1. **Intégrer avec roues** - Attacher 4 moteurs continus aux connecteurs
2. **Simulation complète** - Charger goat_rover_sim.py complètement
3. **Contrôle tendons** - Implémenter la morphologie dynamique
4. **Calibration** - Mesurer réellement E du matériau vs. simulation

---

## 📞 Aide

Pour plus de détails:
- 📄 [README_FRAME_GENESIS.md](./README_FRAME_GENESIS.md) - Configuration détaillée
- 📄 [README_IMPLEMENTATION.md](./README_IMPLEMENTATION.md) - Architecture complète

**Status**: ✅ Opérationnel (FEM + MPM)

