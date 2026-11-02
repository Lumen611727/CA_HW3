# HW3 — Croc's Waterworks: 2D PIC/FLIP Fluid Simulation

Your task is to complete the four core fluid-simulation functions in
`src/fluid.cpp`.

## 1. Before You Start

Open `main.cpp` and replace:

```cpp
constexpr char STUDENT_ID[] = "YOUR_STUDENT_ID";
```

with your own student ID. Rebuild the project after changing it. Your ID appears
in the operating-system window title bar.

## 2. Build

Requirements:

- CMake
- A C++17 compiler
- OpenGL 3.3

**Windows 10/11 is the officially supported environment for this assignment.**
GLFW, GLAD, ImGui, and Eigen are already included under `vendor/`.

### Windows

```powershell
.\build.bat
.\run.bat
```

or manually:

```powershell
cmake -S . -B build
cmake --build build --config Release
```

### Linux / macOS

Linux and macOS are not the primary grading environments. You may use them,
but GLFW can require additional system development packages depending on your
distribution / toolchain.

```sh
cmake -S . -B build
cmake --build build --config Release
./build/hw3
```

Depending on your CMake generator, the Windows executable may be
`build/Release/hw3.exe` or `build/hw3.exe`.

## 3. Required TODOs

Search for `TODO 1`, `TODO 2`, `TODO 3`, and `TODO 4` in `src/fluid.cpp`.

### TODO 1 — `particleRelaxation()`

Implement particle-particle separation using the provided relaxation lookup
grid.

You must:

- assign particles to relaxation cells,
- inspect only the 3 × 3 neighboring cells,
- detect overlaps when the particle distance is less than `2 * radius`,
- move both particles by equal and opposite corrections,
- avoid processing the same particle pair twice,
- handle exact overlap without dividing by zero,
- resolve particle/terrain collision after relaxation.

For particles `p` and `q`, let `x_p` and `x_q` be their positions:

```text
d = x_q - x_p
minimum_distance = 2 * particle_radius
```

When `0 < |d| < minimum_distance`, apply a symmetric positional correction:

```text
correction = 0.5 * (minimum_distance - |d|) * normalize(d)

x_p -= correction
x_q += correction
```

Handle exact overlap separately so that `normalize(d)` does not divide by zero.

### TODO 2 — `transferVelocities(bool to_cell)`

Implement both directions of the PIC/FLIP velocity transfer.

You must:

- reset the MAC-grid velocity field,
- compute staggered-grid bilinear interpolation samples,
- accumulate Particle → Grid velocities and weights,
- normalize grid velocities,
- save the pre-projection grid velocity for FLIP,
- implement the MAC-face validity test used during Grid → Particle interpolation,
- compute both PIC and FLIP velocities,
- blend them with `flip_ratio`.

#### Staggered MAC Grid

For cell `(row, col)`:

```text
Vx(row, col): left face,   offset (0.0, 0.5)
Vy(row, col): bottom face, offset (0.5, 0.0)
```

Offsets are measured in cell units. The two velocity components therefore use
different interpolation grids.

#### Bilinear Weights

For a point surrounded by four samples, let `fx` and `fy` be its local
fractional coordinates in `[0, 1]`:

```text
w00 = (1 - fx) * (1 - fy)
w10 = fx * (1 - fy)
w11 = fx * fy
w01 = (1 - fx) * fy
```

Compute these coordinates relative to the sample grid for each component.
The same weight formulas are also used for density estimation in TODO 3.

#### Particle → Grid

Accumulate each particle's contribution into the four surrounding samples:

```text
grid_velocity += particle_velocity * weight
weight_sum    += weight
```

Then divide each grid velocity sample by its accumulated `weight_sum` when the
sum is nonzero. Save the normalized field to `prev_cell_velocities` before
incompressibility projection.

#### Grid → Particle

First implement `isFaceVelocityValid(row, col, component)`. A staggered face
is valid for interpolation when at least one of its adjacent cells is FLUID.
Then use this helper to reject invalid samples and renormalize the remaining
weights. For each velocity component:

```text
v_pic = sum(w_i * current_grid_velocity_i) / sum(w_i)

v_flip = old_particle_velocity
       + sum(w_i * (current_grid_velocity_i - previous_grid_velocity_i))
         / sum(w_i)

v = (1 - flip_ratio) * v_pic + flip_ratio * v_flip
```

The sums include only valid samples. Guard against a zero weight sum.
`previous_grid_velocity_i` comes from `prev_cell_velocities`.

### TODO 3 — `updateDensity()`

Estimate density at cell centers.

You must:

- reset density every frame,
- use bilinear interpolation at cell-center sample positions,
- accumulate particle contributions,
- ignore SOLID cells,
- initialize `particle_rest_density` from the average FLUID-cell density.

Density samples use a `(0.5, 0.5)` offset in cell units. Each particle
contributes its four bilinear weights to the surrounding non-SOLID samples:

```text
cell_density += weight
```

On the first density update, initialize:

```text
particle_rest_density =
    sum(density of FLUID cells) / number_of_FLUID_cells
```

Guard against an empty FLUID-cell set before dividing.

### TODO 4 — `solveIncompressibility()`

Implement the iterative incompressibility projection.

For each FLUID cell:

- count available non-SOLID flow directions,
- compute MAC-grid divergence,
- optionally add the density-compression correction,
- divide the correction among available directions,
- apply over-relaxation,
- update the four MAC face velocities with the correct signs.

Using the MAC-grid convention, compute:

```text
left   = Vx(row,     col)
right  = Vx(row,     col + 1)
bottom = Vy(row,     col)
top    = Vy(row + 1, col)

divergence = top + right - left - bottom
```

If `density_correction` is enabled and the cell is over-compressed:

```text
compression = cell_density - particle_rest_density
divergence -= stiffness_coefficient * compression
```

Only positive compression contributes. Count the directions whose neighboring
cells are non-SOLID, and skip the cell if no direction is available:

```text
correction = over_relaxation * divergence / number_of_open_directions
```

Apply each correction only when its corresponding direction is available:

```text
left   += correction
right  -= correction
bottom += correction
top    -= correction
```

Repeat for the configured solver iterations to drive divergence toward zero.

Detailed implementation guidance and formulas are written directly above each
TODO in `src/fluid.cpp`.

## 4. Controls

- Left mouse drag: dig terrain
- Mouse wheel: change dig radius
- `Space`: pause / resume
- `R`: restart the level
- `1`: particles
- `2`: grid cells
- `3`: particles + grid
- Simulation parameters can be changed from the right-side control panel.

## 5. Goal

Dig a route from the initial water reservoir to the `PIPE INLET`.

Water entering the pipe is emitted from the shower. Only water that has passed
through the pipe and falls into the bathtub counts toward the score.

- 70% collected: Level Clear / 1 star
- 85% collected: 2 stars
- 95% collected: 3 stars

## 6. Self-Test

The project includes a headless test mode:

```powershell
ctest --test-dir build -C Release --output-on-failure
```

The starter code is expected to fail the fluid-pipeline test before the TODOs
are implemented. Use the tests as one debugging signal; also inspect the
simulation visually.

## 7. Recommended Debugging Order

Implement and debug the TODOs in this order:

1. `particleRelaxation()`
2. Particle → Grid transfer in `transferVelocities(true)`
3. `updateDensity()`
4. `solveIncompressibility()`
5. Grid → Particle transfer in `transferVelocities(false)`

Useful symptoms:

- particles overlap heavily → check TODO 1
- grid shows almost no velocity → check Particle → Grid transfer
- fluid compresses into dense clumps → check density / incompressibility
- particles become unstable after projection → check Grid → Particle and FLIP
- fluid explodes → check divergence signs and correction signs

## 8. Files You Normally Need to Modify

```text
main.cpp          # student ID only
src/fluid.cpp     # required implementation TODOs
```

Do not modify rendering, scene geometry, pipe / shower logic, scoring, or other
game logic unless explicitly allowed as an optional extension.

## 9. Report

Use `REPORT_TEMPLATE.md` as the recommended structure for your report. Convert
or export your completed report to PDF before submission.

Your report should contain the following sections.

### 9.1 Implementation

Explain the reasoning behind your implementation of:

- Particle Relaxation
- PIC/FLIP Particle ↔ Grid Velocity Transfer
- Cell Density Estimation and Rest Density
- Incompressibility Projection and Density Correction

Do not only paste code. Explain the algorithm, indexing / interpolation choices,
and any special cases you handled.

### 9.2 Results & Discussion

Experiment with **at least three** simulation parameters and discuss how they
affect the resulting fluid behavior. You may choose from:

- FLIP ratio
- Time step (`dt`)
- Stiffness coefficient
- Solver iterations
- Over-relaxation
- Gravity

For each experiment, include the tested values, at least one screenshot, and a
short discussion of the observed behavior. You may additionally experiment with
other parameters if they are relevant to your implementation.

### 9.3 Final Demo Result

Include a screenshot of your final working simulation showing:

- your student ID in the window title,
- the water successfully routed through the pipe / shower,

A separate demo video is **not required** unless announced otherwise by the TA.

### 9.4 Bonus / Miscellaneous (Optional)

Briefly describe any creative extension, additional experiment, variable change,
or function rewrite that is outside the required TODO sections.

## 10. Grading

The grading scheme follows the previous 2D Fluid Simulation HW3 structure:

| Item | Points |
| --- | ---: |
| `particleRelaxation()` | 15 |
| `transferVelocities()` + `isFaceVelocityValid()` | 25 |
| └ Particle → Grid | 10 |
| └ Grid → Particle / PIC-FLIP / valid-face handling | 15 |
| `updateDensity()` + density correction | 15 |
| └ Cell densities | 10 |
| └ Rest density | 2 |
| └ Density correction in incompressibility | 3 |
| `solveIncompressibility()` | 25 |
| Report | 20 |
| **Total** | **100** |

**Bonus:** up to **+15 points** for meaningful creativity or extensions, subject
to TA evaluation.

## 11. Submission

Submit a zip file named:

```text
HW3_<studentID>.zip
```

with the following structure:

```text
HW3_<studentID>.zip/
├── main.cpp
├── src/
│   └── fluid.cpp
└── report_<studentID>.pdf
```

Only submit the files listed above unless the TA explicitly asks for additional
files. Do **not** include the `build/` directory, executables, CMake cache files,
or other generated build artifacts.

## 12. If You Get Stuck, You Can Feed the Cute Little Hamster

```text
　　　､ゞヾ∧""'"∧;,
　　ヾ　　　・ω・   彡
　 ﾐ　　   o 🍎 o  ミ
　~彡　　　　　　　  ﾐ
　　/ｿ,,,０;,;;:､０ヾ`
```
