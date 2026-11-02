# HW3 Report — Croc's Waterworks: 2D PIC/FLIP Fluid Simulation

**Student Name:** YOUR_NAME  
**Student ID:** YOUR_STUDENT_ID  

> Replace all placeholder text before submission. Keep the report concise, but
> include enough explanation and evidence for the TA to understand and verify
> your implementation.

---

## 1. Implementation

### 1.1 TODO 1 — Particle Relaxation

Describe how you implemented `particleRelaxation()`.

Please discuss:

- how particles are assigned to the relaxation lookup grid,
- why only a 3 × 3 neighborhood needs to be searched,
- how overlapping particles are detected,
- how the symmetric position correction is computed,
- how you avoid processing the same particle pair twice,
- how you handle the exact-overlap / zero-distance case,
- how terrain collision is handled after relaxation.

**Implementation summary:**  

```text
YOUR_EXPLANATION_HERE
```

---

### 1.2 TODO 2 — PIC/FLIP Velocity Transfer

Describe your implementation of `isFaceVelocityValid()` and
`transferVelocities(bool to_cell)`.

Please discuss:

- the staggered MAC-grid layout,
- why Vx and Vy use different sampling offsets,
- how the four bilinear interpolation weights are computed,
- Particle → Grid accumulation and weight normalization,
- when and why `prev_cell_velocities` is saved,
- how valid / invalid MAC faces are determined,
- how PIC velocity is computed,
- how FLIP velocity is computed,
- how `flip_ratio` blends PIC and FLIP.

**Implementation summary:**  
```text
YOUR_EXPLANATION_HERE
```

---

### 1.3 TODO 3 — Cell Density Estimation

Describe your implementation of `updateDensity()`.

Please discuss:

- why density is sampled at cell centers,
- how a particle contributes to four surrounding density samples,
- how SOLID cells are handled,
- how `particle_rest_density` is initialized,
- why the rest density is computed only once.

**Implementation summary:**  
```text
YOUR_EXPLANATION_HERE
```

---

### 1.4 TODO 4 — Incompressibility Solver

Describe your implementation of `solveIncompressibility()`.

Please discuss:

- how divergence is computed from the four MAC-face velocities,
- how SOLID neighbors affect the number of available flow directions,
- how the divergence correction is distributed,
- how `over_relaxation` is used,
- how density correction is added when a cell is over-compressed,
- why the solver is repeated for multiple iterations.

**Implementation summary:**  
```text
YOUR_EXPLANATION_HERE
```

---

## 2. Results & Discussion

Perform the following **three required experiments**.

For each experiment:

- keep all other simulation parameters at their default values,
- change only the specified parameter,
- run the same scene under each setting,
- include screenshots if they help illustrate the difference,
- describe the visible behavior and explain why the parameter causes that effect.

### 2.1 Experiment 1 — FLIP Ratio

Test the following values:

- `0.0`
- `0.5`
- `0.95`

**Other parameters:** kept at default values.

| FLIP Ratio | Observation |
| --- | --- |
| 0.0 | YOUR_OBSERVATION |
| 0.5 | YOUR_OBSERVATION |
| 0.95 | YOUR_OBSERVATION |

**Discussion:**  
Compare the motion produced by lower and higher FLIP ratios. Discuss effects such
as smoothness, damping, preservation of motion, splashing, and numerical noise.

```text
YOUR_DISCUSSION_HERE
```

---

### 2.2 Experiment 2 — Solver Iterations

Test the following values:

- `1`
- `3`
- `8`

**Other parameters:** kept at default values.

| Solver Iterations | Observation |
| --- | --- |
| 1 | YOUR_OBSERVATION |
| 3 | YOUR_OBSERVATION |
| 8 | YOUR_OBSERVATION |

**Discussion:**  
Compare how the number of incompressibility-solver iterations affects fluid
compression, volume preservation, visual stability, and computational cost.

```text
YOUR_DISCUSSION_HERE
```

---

### 2.3 Experiment 3 — Stiffness Coefficient

Test the following values:

- `0.0`
- `1.0`
- `3.0`

**Other parameters:** kept at default values.

| Stiffness Coefficient | Observation |
| --- | --- |
| 0.0 | YOUR_OBSERVATION |
| 1.0 | YOUR_OBSERVATION |
| 3.0 | YOUR_OBSERVATION |

**Discussion:**  
Compare how the stiffness coefficient affects density correction. Discuss
particle compression, spreading behavior, stability, and any visible artifacts
when the coefficient becomes larger.

```text
YOUR_DISCUSSION_HERE
```

---

## 3. Final Demo Result

Include at least one screenshot showing that your completed implementation can
successfully guide water through the level and reach the bathtub.

Briefly describe whether the simulation behaves as expected and mention any
remaining visual or numerical issues you observed.


## 4. Bonus / Miscellaneous (Optional)

If you implemented any creative extension or changed code outside the required
TODO sections, describe it here.

Examples:

- additional visual effects,
- new interaction / gameplay behavior,
- alternative numerical method,
- performance optimization,
- additional experiment or visualization.

**Description:**  
```text
YOUR_DISCUSSION_HERE
```

**Screenshot / evidence (if applicable):**  
```text
YOUR_SCREENSHOT_HERE
```

---

## 5. References (Optional)

List any external resources that materially helped your implementation. Follow
the course academic-integrity policy and clearly identify external code or ideas
when applicable.

- REFERENCE_1
- REFERENCE_2
