#include "fluid.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace
{
constexpr float EPS = 1e-6f;

// NOTE FOR STUDENTS:
// The original reference solution used private helper functions for bilinear
// interpolation here. They are intentionally omitted from the starter code.
// Computing the four staggered-grid sample locations and their bilinear weights
// is part of TODO 2 (velocity transfer) and TODO 3 (density estimation).

inline float clampf(float x, float a, float b)
{
    return std::max(a, std::min(b, x));
}
}

void Fluid::integrate()
{
    Eigen::Vector2f gravityVec(0.0f, -gravity * dt);
    for (int i = 0; i < num_particles; ++i)
    {
        particle_vel[i] += gravityVec;
        particle_pos[i] += dt * particle_vel[i];
    }
}

void Fluid::handleCollisions()
{
    for (int i = 0; i < num_particles; ++i)
        resolveParticleTerrainCollision(particle_pos[i], particle_vel[i]);
}

void Fluid::particleRelaxation()
{
    /*
    ============================================================================
    TODO 1 — Particle Relaxation
    ============================================================================

    Goal
    ----
    Prevent fluid particles from occupying the same space. The simulation uses
    a uniform spatial lookup grid (`relaxation_cell_particle_ids`) so that each
    particle only needs to check nearby particles instead of all N^2 pairs.

    What you need to implement
    --------------------------
    A. Assign every particle to one relaxation cell.

       For a particle at position (x, y), compute its cell coordinate using

           col = floor(x / relaxation_cell_dim)
           row = floor(y / relaxation_cell_dim)

       Clamp the indices to the valid range before inserting the particle index
       into `relaxation_cell_particle_ids[row][col]`.

    B. Perform the iterative relaxation solver.

       For each solver iteration:
         1. Visit every relaxation cell.
         2. For each particle p in that cell, inspect particles in the 3 x 3
            neighborhood centered at the current cell.
         3. Avoid solving the same pair twice. One simple rule is to ignore q
            when q <= p.
         4. Let

                d = position[q] - position[p]
                min_distance = 2 * particle_radius

            The pair overlaps when |d| < min_distance.

         5. Move the two particles by equal and opposite corrections so that
            they are separated by `min_distance`.

            A typical correction is

                correction =
                    0.5 * (min_distance - |d|) * normalize(d)

                position[p] -= correction
                position[q] += correction

         6. Handle the special case |d| ~= 0. Two particles may be exactly on
            top of each other, so normalizing d directly would divide by zero.

    C. Terrain interaction

       Relaxation can push particles into solid terrain. After each relaxation
       iteration, call `resolveParticleTerrainCollision(...)` for every particle
       so that the game scene remains valid.

    Useful members
    --------------
      relaxation_cell_rows
      relaxation_cell_cols
      relaxation_cell_dim
      relaxation_cell_particle_ids

      particle_pos
      particle_vel
      particle_radius
      num_particles
      iterations

    Debugging hints
    ---------------
    - Render particles and watch for pairs that remain stacked.
    - If the fluid suddenly explodes, check the sign and magnitude of the
      position correction.
    - If performance is very poor, make sure you are using the 3 x 3 lookup
      instead of comparing every particle against every other particle.
    */

    // Clear the lookup grid before assigning particles.
    for (auto& row : relaxation_cell_particle_ids)
        for (auto& ids : row)
            ids.clear();

    // TODO 1A: Assign each particle index to its relaxation cell.
    for (int p = 0; p < num_particles; ++p)
    {
        int col = std::floor(particle_pos[p].x() / relaxation_cell_dim);
        int row = std::floor(particle_pos[p].y() / relaxation_cell_dim);
        col = clampf(col, 0, relaxation_cell_cols - 1);
        row = clampf(row, 0, relaxation_cell_rows - 1);

        relaxation_cell_particle_ids[row][col].push_back(p);
    }

    for (int iter = 0; iter < iterations; ++iter)
    {
        for (int r = 0; r < relaxation_cell_rows; ++r)
        {
            for (int c = 0; c < relaxation_cell_cols; ++c)
            {
                // TODO 1B: Resolve overlapping particle pairs in the 3 x 3
                // neighborhood around relaxation cell (r, c).
                for (int p : relaxation_cell_particle_ids[r][c])
                {
                    for (int dr = -1; dr <= 1; ++dr)
                    {
                        for (int dc = -1; dc <= 1; ++dc)
                        {
                            int nr = r + dr;
                            int nc = c + dc;
                            if (nr < 0 || nr >= relaxation_cell_rows || nc < 0 || nc >= relaxation_cell_cols)
                                continue;

                            for (int q : relaxation_cell_particle_ids[nr][nc])
                            {
                                if (q <= p) continue;

                                Eigen::Vector2f d = particle_pos[q] - particle_pos[p];
                                float min_distance = 2 * particle_radius;
                                float dist = d.norm();

                                if (dist < min_distance && dist > EPS)
                                {
                                    Eigen::Vector2f correction = 0.5f * (min_distance - dist) * d.normalized();
                                    particle_pos[p] -= correction;
                                    particle_pos[q] += correction;
                                }
                                else if (dist < min_distance && dist <= EPS)
                                {
                                    Eigen::Vector2f fixed_dir(1.0f, 0.0f); 
                                    Eigen::Vector2f correction = 0.5f * min_distance * fixed_dir;
                                    particle_pos[p] -= correction;
                                    particle_pos[q] += correction;
                                }
                            }
                        }
                    }
                }
            }
        }

        // TODO 1C: Re-resolve particle/terrain collision after the positional
        // relaxation step.
        handleCollisions();
    }
}

void Fluid::applyTerrainTypes()
{
    for (int r = 0; r < cell_rows; ++r)
    {
        for (int c = 0; c < cell_cols; ++c)
            cell_types[r][c] = isTerrainSolid(r, c) ? CellType::SOLID : CellType::AIR;
    }

    // Any non-solid cell containing a particle becomes FLUID.
    for (int p = 0; p < num_particles; ++p)
    {
        int c = static_cast<int>(particle_pos[p].x() / cell_dim);
        int r = static_cast<int>(particle_pos[p].y() / cell_dim);
        if (r >= 0 && r < cell_rows && c >= 0 && c < cell_cols && cell_types[r][c] != CellType::SOLID)
            cell_types[r][c] = CellType::FLUID;
    }
}

bool Fluid::isFaceVelocityValid(int row, int col, int component) const
{
    /*
    ============================================================================
    TODO 2A — Valid MAC-Face Test
    ============================================================================

    This helper is part of the PIC/FLIP velocity-transfer task.

    A staggered MAC velocity sample lies on a FACE between two cells. During
    Grid -> Particle interpolation, a face should be used only when it is
    adjacent to at least one FLUID cell.

    Steps
    -----
    1. Reject sample coordinates that are outside the MAC-grid array.

    2. For the x component (component == 0):

           Vx(row, col)

       is the vertical face between cells

           (row, col - 1)   and   (row, col).

    3. For the y component (component == 1):

           Vy(row, col)

       is the horizontal face between cells

           (row - 1, col)   and   (row, col).

    4. Return true when at least one valid adjacent cell is FLUID.
       Otherwise return false.

    Be careful at the left and bottom boundaries: one of the adjacent cells may
    not exist, so check its index before reading `cell_types`.
    */

    // TODO(student): Implement the MAC-face validity test described above.
    if (row < 0 || row >= cell_rows || col < 0 || col >= cell_cols) return false;
    int nr = (component == 1) ? row - 1 : row;
    int nc = (component == 0) ? col - 1 : col;
    bool exist_fluid_neighbor = (nr >= 0 && nc >= 0) && cell_types[nr][nc] == CellType::FLUID;
    bool self_fluid = cell_types[row][col] == CellType::FLUID;
    return exist_fluid_neighbor || self_fluid;
}

void Fluid::transferVelocities(bool to_cell)
{
    /*
    ============================================================================
    TODO 2 — PIC/FLIP Velocity Transfer
    ============================================================================

    Goal
    ----
    Transfer velocity between particles and the staggered MAC grid.

    The parameter `to_cell` selects the direction:
      - true  : particle velocity -> MAC grid
      - false : MAC grid velocity -> particle velocity

    MAC-grid convention used in this assignment
    -------------------------------------------
      cell_velocities[row][col].x() = Vx sampled on the LEFT face of the cell
      cell_velocities[row][col].y() = Vy sampled on the BOTTOM face of the cell

    Therefore the two components use different sample offsets.

    For Vx:
      sample points are located at integer x and half-cell y positions.
      Think of the staggered-grid offset as (0, 0.5).

    For Vy:
      sample points are located at half-cell x and integer y positions.
      Think of the staggered-grid offset as (0.5, 0).

    For each particle and each velocity component, find the four surrounding
    grid samples and compute bilinear interpolation weights w1..w4.

    --------------------------------------------------------------------------
    Part A — Particle -> Grid
    --------------------------------------------------------------------------

    1. The scene-specific cell classification is already provided by
       `applyTerrainTypes()`. It marks editable terrain / fixed walls as SOLID
       and cells containing particles as FLUID.

    2. Reset `cell_velocities` to zero.

    3. Create a weight-accumulation buffer with the same shape as
       `cell_velocities`.

    4. For each particle and component:
         grid_velocity += particle_velocity * weight
         weight_sum    += weight

    5. After all particles have contributed, normalize each non-empty grid
       velocity sample by its accumulated weight.

    6. Copy the normalized velocity field to `prev_cell_velocities`.
       This backup MUST happen before `solveIncompressibility()` modifies the
       grid because FLIP uses the change in grid velocity.

    --------------------------------------------------------------------------
    Part B — Grid -> Particle
    --------------------------------------------------------------------------

    For each particle and component:

    1. Recompute the same four staggered-grid samples and bilinear weights.

    2. Ignore invalid velocity samples. The provided helper

           isFaceVelocityValid(row, col, component)

       returns true when a staggered face is adjacent to at least one FLUID
       cell. Renormalize the remaining valid weights.

    3. PIC velocity

           v_pic = sum(w_i * v_i) / sum(w_i)

    4. FLIP velocity

           v_flip =
               v_particle_old
               + sum(w_i * (v_i - v_i_previous)) / sum(w_i)

       where `v_i_previous` comes from `prev_cell_velocities`.

    5. Blend the two:

           v_particle =
               (1 - flip_ratio) * v_pic
               + flip_ratio * v_flip

    Useful members / helpers
    ------------------------
      particle_pos
      particle_vel
      num_particles

      cell_rows
      cell_cols
      cell_dim
      cell_velocities
      prev_cell_velocities
      cell_types

      flip_ratio
      applyTerrainTypes()
      isFaceVelocityValid(...)

    Debugging hints
    ---------------
    - Do not use the same geometric offset for Vx and Vy.
    - Always renormalize after rejecting invalid samples.
    - If FLIP behaves wildly, verify that `prev_cell_velocities` stores the
      pre-projection grid velocity, not the post-projection velocity.
    */

    // Weight normalization buffer for Particle -> Grid transfer.
    std::vector<std::vector<Eigen::Vector2f>> weight_sum(
        cell_rows,
        std::vector<Eigen::Vector2f>(cell_cols, Eigen::Vector2f::Zero()));

    if (to_cell)
    {
        // Provided scene setup. Do not replace terrain/fixed-wall cells here.
        applyTerrainTypes();

        // TODO 2B: Reset all MAC-grid velocity samples to zero.
        for (int r = 0; r < cell_rows; ++r)
            for (int c = 0; c < cell_cols; ++c)
                cell_velocities[r][c] = Eigen::Vector2f::Zero();
    }

    for (int component = 0; component < 2; ++component)
    {
        for (int p = 0; p < num_particles; ++p)
        {
            /*
            TODO 2C: Bilinear interpolation on the staggered grid.

            Compute:
              - the four sample row indices,
              - the four sample column indices,
              - the four bilinear weights.

            Remember:
              component == 0 -> Vx offset (0.0, 0.5)
              component == 1 -> Vy offset (0.5, 0.0)

            You may use local arrays such as:
              int sample_row[4];
              int sample_col[4];
              float weight[4];
            */
            int sample_row[4];
            int sample_col[4];
            float weight[4];

            float ox = (component == 0) ? 0.0f : 0.5f;
            float oy = (component == 0) ? 0.5f : 0.0f;
            float gx = particle_pos[p].x() / cell_dim - ox;
            float gy = particle_pos[p].y() / cell_dim - oy;
            int c0 = clampf(std::floor(gx), 0, cell_cols - 2);
            int r0 = clampf(std::floor(gy), 0, cell_rows - 2);
            float fx = clampf(gx - c0, 0.0f, 1.0f);
            float fy = clampf(gy - r0, 0.0f, 1.0f);

            sample_row[0] = r0;
            sample_col[0] = c0;
            weight[0] = (1.0f - fx) * (1.0f - fy);
            sample_row[1] = r0;
            sample_col[1] = c0 + 1;
            weight[1] = fx * (1.0f - fy);
            sample_row[2] = r0 + 1;
            sample_col[2] = c0;
            weight[2] = (1.0f - fx) * fy;
            sample_row[3] = r0 + 1;
            sample_col[3] = c0 + 1;
            weight[3] = fx * fy;

            if (to_cell)
            {
                // TODO 2D: Accumulate particle velocity and interpolation
                // weights into the four MAC-grid samples.
                for (int i = 0; i < 4; ++i)
                {
                    int r = sample_row[i];
                    int c = sample_col[i];
                    if (r >= 0 && r < cell_rows && c >= 0 && c < cell_cols)
                    {
                        cell_velocities[r][c][component] += particle_vel[p][component] * weight[i];
                        weight_sum[r][c][component] += weight[i];
                    }
                }
            }
            else
            {
                // TODO 2E: Interpolate only valid MAC-grid samples, compute
                // PIC and FLIP velocities, then blend them using flip_ratio.
                float pic_velocity = 0.0f;
                float flip_velocity = 0.0f;
                float total_weight = 0.0f;

                for (int i = 0; i < 4; ++i)
                {
                    int r = sample_row[i];
                    int c = sample_col[i];
                    if (r >= 0 && r < cell_rows && c >= 0 && c < cell_cols && isFaceVelocityValid(r, c, component))
                    {
                        total_weight += weight[i];
                        pic_velocity += cell_velocities[r][c][component] * weight[i];
                        flip_velocity += (cell_velocities[r][c][component] - prev_cell_velocities[r][c][component]) * weight[i];
                    }
                }
                
                if(total_weight > 0.0f)
                {
                    pic_velocity /= total_weight;
                    flip_velocity /= total_weight;
                    flip_velocity += particle_vel[p][component];
                    particle_vel[p][component] = (1.0f - flip_ratio) * pic_velocity + flip_ratio * flip_velocity;
                }
            }
        }
    }

    if (to_cell)
    {
        // TODO 2F: Normalize cell velocities by weight_sum.
        for (int r = 0; r < cell_rows; ++r)
        {
            for (int c = 0; c < cell_cols; ++c)
            {
                if (weight_sum[r][c].x() > 0.0f)
                    cell_velocities[r][c].x() /= weight_sum[r][c].x();
                if (weight_sum[r][c].y() > 0.0f)
                    cell_velocities[r][c].y() /= weight_sum[r][c].y();
            }
        }
        // TODO 2G: Save the normalized pre-projection grid velocity into
        // prev_cell_velocities for the later FLIP update.
        prev_cell_velocities = cell_velocities;
    }
}

void Fluid::updateDensity()
{
    /*
    ============================================================================
    TODO 3 — Cell Density Estimation and Rest Density
    ============================================================================

    Goal
    ----
    Estimate particle density at cell centers. The incompressibility solver
    uses this density to reduce excessive particle compression.

    Cell-center sampling
    --------------------
    Unlike MAC velocity samples, density is sampled at the CENTER of each cell.
    Therefore both axes use a half-cell offset:

        density sample offset = (0.5, 0.5)

    For every particle:
      1. Find the four surrounding cell-center sample points.
      2. Compute the bilinear weights w1..w4.
      3. Add 1 * w_i to each valid non-SOLID density sample.

    Rest density
    ------------
    `particle_rest_density` is initialized to zero. Compute it only on the
    first density update:

        rest_density =
            sum(density of FLUID cells) / number_of_FLUID_cells

    The initial particle layout is intentionally densely packed so this average
    provides the reference density used by the correction term.

    Useful members
    --------------
      particle_pos
      num_particles

      cell_rows
      cell_cols
      cell_dim
      cell_types
      cell_densities

      particle_rest_density

    Debugging hints
    ---------------
    - Reset all `cell_densities` before accumulating the new frame.
    - Do not write density into SOLID cells.
    - Protect against an empty FLUID-cell set when computing the average.
    */

    // Reset density values every frame.
    for (int r = 0; r < cell_rows; ++r)
        for (int c = 0; c < cell_cols; ++c)
            cell_densities[r][c] = 0.0f;

    for (int p = 0; p < num_particles; ++p)
    {
        // TODO 3A: Compute cell-center bilinear weights and accumulate this
        // particle's contribution into the four surrounding density samples.
        int sample_row[4];
        int sample_col[4];
        float weight[4];

        float gx = particle_pos[p].x() / cell_dim - 0.5f;
        float gy = particle_pos[p].y() / cell_dim - 0.5f;
        int c0 = clampf(std::floor(gx), 0, cell_cols - 2);
        int r0 = clampf(std::floor(gy), 0, cell_rows - 2);
        float fx = clampf(gx - c0, 0.0f, 1.0f);
        float fy = clampf(gy - r0, 0.0f, 1.0f);

        sample_row[0] = r0;
        sample_col[0] = c0;
        weight[0] = (1.0f - fx) * (1.0f - fy);
        sample_row[1] = r0;
        sample_col[1] = c0 + 1;
        weight[1] = fx * (1.0f - fy);
        sample_row[2] = r0 + 1;
        sample_col[2] = c0;
        weight[2] = (1.0f - fx) * fy;
        sample_row[3] = r0 + 1;
        sample_col[3] = c0 + 1;
        weight[3] = fx * fy;

        for (int i = 0; i < 4; ++i)
        {
            int r = sample_row[i];
            int c = sample_col[i];
            if (r >= 0 && r < cell_rows && c >= 0 && c < cell_cols && cell_types[r][c] != CellType::SOLID)
            {
                cell_densities[r][c] += weight[i];
            }
        }
    }

    if (particle_rest_density <= 0.0f)
    {
        // TODO 3B: Compute the average density of FLUID cells and store it in
        // particle_rest_density. This should only be initialized once.
        float total_density = 0.0f;
        int fluid_cell_count = 0;
        for (int r = 0; r < cell_rows; ++r)
        {
            for (int c = 0; c < cell_cols; ++c)
            {
                if (cell_types[r][c] == CellType::FLUID)
                {
                    total_density += cell_densities[r][c];
                    fluid_cell_count++;
                }
            }
        }
        if (fluid_cell_count > 0)
        {
            particle_rest_density = total_density / fluid_cell_count;
        }
    }
}

void Fluid::solveIncompressibility()
{
    /*
    ============================================================================
    TODO 4 — Incompressibility Solver
    ============================================================================

    Goal
    ----
    Iteratively modify MAC-grid velocities so that fluid cells have divergence
    close to zero. This approximates incompressible water.

    For every solver iteration and every FLUID cell:

    A. Determine which of the four neighboring directions are available.

       A direction is available when the neighboring cell is NOT SOLID.

       Count:
         left, right, bottom, top

       If no direction is available, skip the cell.

    B. Compute divergence using the MAC-grid convention:

         left   = Vx(row,     col)
         right  = Vx(row,     col + 1)
         bottom = Vy(row,     col)
         top    = Vy(row + 1, col)

         divergence = top + right - left - bottom

       Positive/negative sign mistakes here usually make the fluid explode.

    C. Optional density correction.

       If density correction is enabled and the current cell is denser than
       the rest density:

         compression = cell_density - particle_rest_density

       Add an expansion bias by modifying divergence as

         divergence -= stiffness_coefficient * compression

       Only positive compression should contribute.

    D. Compute the correction magnitude.

         correction =
             over_relaxation * divergence / available_direction_count

       `over_relaxation` accelerates convergence. The default value is chosen
       for this assignment, so do not hard-code a different constant.

    E. Apply equal/opposite corrections to the four MAC faces:

         left   += correction
         right  -= correction
         bottom += correction
         top    -= correction

       Only modify a face when the corresponding direction is available.

    Useful members
    --------------
      iterations
      cell_rows
      cell_cols
      cell_types
      cell_velocities
      cell_densities

      density_correction
      particle_rest_density
      stiffness_coefficient
      over_relaxation

    Debugging hints
    ---------------
    - Process FLUID cells only.
    - Never divide by zero when a cell has no available direction.
    - If water rapidly gains energy, first check the divergence signs and the
      correction signs.
    */

    for (int iter = 0; iter < iterations; ++iter)
    {
        for (int r = 1; r < cell_rows - 1; ++r)
        {
            for (int c = 1; c < cell_cols - 1; ++c)
            {
                // TODO 4A: Skip non-FLUID cells and count non-SOLID neighbor
                // directions.
                if (cell_types[r][c] != CellType::FLUID) continue;

                int available_cell_count = 0;
                for (int dr = -1; dr <= 1; ++dr)
                {
                    for (int dc = -1; dc <= 1; ++dc)
                    {
                        if (std::abs(dr) + std::abs(dc) != 1) continue;
                        int nr = r + dr;
                        int nc = c + dc;
                        if (nr >= 0 && nr < cell_rows && nc >= 0 && nc < cell_cols && cell_types[nr][nc] != CellType::SOLID)
                        {
                            available_cell_count++;
                        }
                    }
                }

                // TODO 4B: Compute MAC-grid divergence.
                float left = cell_velocities[r][c].x();
                float right = cell_velocities[r][c + 1].x();
                float bottom = cell_velocities[r][c].y();
                float top = cell_velocities[r + 1][c].y();
                float divergence = top + right - left - bottom;

                // TODO 4C: Apply density correction when enabled.
                if (density_correction && particle_rest_density > 0.0f && cell_densities[r][c] > particle_rest_density)
                {
                    float compression = cell_densities[r][c] - particle_rest_density;
                    divergence -= stiffness_coefficient * compression;
                }

                // TODO 4D: Compute the over-relaxed correction magnitude.
                float correction = 0.0f;
                if (available_cell_count > 0)
                {
                    correction = over_relaxation * divergence / available_cell_count;
                }

                // TODO 4E: Apply the correction to available MAC faces.
                if (available_cell_count > 0)
                {
                    if (cell_types[r][c - 1] != CellType::SOLID)
                        cell_velocities[r][c].x() += correction;
                    if (cell_types[r][c + 1] != CellType::SOLID)
                        cell_velocities[r][c + 1].x() -= correction;
                    if (cell_types[r - 1][c] != CellType::SOLID)
                        cell_velocities[r][c].y() += correction;
                    if (cell_types[r + 1][c] != CellType::SOLID)
                        cell_velocities[r + 1][c].y() -= correction;
                }
            }
        }
    }
}

Fluid::Fluid(int num_particles,
             float radius,
             float flip_ratio,
             float cell_dim,
             float relaxation_cell_dim,
             int iterations,
             float viewport_w,
             float viewport_h,
             float dt,
             float gravity,
             float stiffness,
             bool density_correction,
             float over_relaxation)
    : iterations(iterations),
      dt(dt),
      gravity(gravity),
      viewport_w(viewport_w),
      viewport_h(viewport_h),
      flip_ratio(flip_ratio),
      stiffness_coefficient(stiffness),
      over_relaxation(over_relaxation),
      density_correction(density_correction),
      particle_radius(radius),
      num_particles(num_particles),
      initial_particle_count(num_particles),
      collected_particles(0),
      particle_rest_density(0.0f),
      relaxation_cell_dim(relaxation_cell_dim),
      cell_dim(cell_dim)
{
    // cell_dim is a required simulation scale. Keep a defensive fallback so
    // a malformed caller cannot create a zero/garbage-sized MAC grid.
    if (!(cell_dim > 0.0f) || !std::isfinite(cell_dim))
        cell_dim = 10.0f;
    this->cell_dim = cell_dim;

    cell_rows = static_cast<int>(viewport_h / cell_dim);
    cell_cols = static_cast<int>(viewport_w / cell_dim);

    relaxation_cell_rows = static_cast<int>(viewport_h / relaxation_cell_dim);
    relaxation_cell_cols = static_cast<int>(viewport_w / relaxation_cell_dim);

    relaxation_cell_particle_ids.assign(
        relaxation_cell_rows,
        std::vector<std::vector<int>>(relaxation_cell_cols));

    cell_types.assign(cell_rows, std::vector<CellType>(cell_cols, CellType::AIR));
    cell_velocities.assign(cell_rows, std::vector<Eigen::Vector2f>(cell_cols, Eigen::Vector2f::Zero()));
    prev_cell_velocities.assign(cell_rows, std::vector<Eigen::Vector2f>(cell_cols, Eigen::Vector2f::Zero()));
    cell_densities.assign(cell_rows, std::vector<float>(cell_cols, 0.0f));
    terrain_solid.assign(cell_rows, std::vector<unsigned char>(cell_cols, 0));
    fixed_wall.assign(cell_rows, std::vector<unsigned char>(cell_cols, 0));

    protected_air.assign(cell_rows, std::vector<unsigned char>(cell_cols, 0));
    particle_pos.assign(num_particles, Eigen::Vector2f::Zero());
    particle_vel.assign(num_particles, Eigen::Vector2f::Zero());
    particle_piped.assign(num_particles, 0);

    resetScene();
}

void Fluid::wallRect(float x0, float y0, float x1, float y1)
{
    for (int r = 0; r < cell_rows; ++r)
        for (int c = 0; c < cell_cols; ++c)
        {
            float x = (c + 0.5f) * cell_dim, y = (r + 0.5f) * cell_dim;
            if (x >= x0 && x < x1 && y >= y0 && y < y1)
                fixed_wall[r][c] = terrain_solid[r][c] = 1;
        }
}
void Fluid::initializeTerrain()
{
    for (int r = 0; r < cell_rows; ++r)
        for (int c = 0; c < cell_cols; ++c)
        {
            float x = (c + 0.5f) * cell_dim, y = (r + 0.5f) * cell_dim;
            terrain_solid[r][c] = (x >= 80 && x < 1020 && y >= 50 && y < 810);
            fixed_wall[r][c] = protected_air[r][c] = 0;
        }
    // Accommodate the original 2600 particles at their original radius and
    // spawn positions; adapt scenery rather than changing the simulation.
    carveRect(110, 500, 710, 790);
    carveRect(650, 570, 790, 650);
    carveCircle(290, 425, 55);
    carveRect(700, 380, 880, 445);
    carveRect(830, 270, 940, 350);
    carveRect(360, 70, 750, 290);
    // Shelves and the replacements for the circle and gate are immutable.
    wallRect(80, 450, 460, 480);
    wallRect(590, 350, 640, 480);
    wallRect(380, 350, 790, 380);
    wallRect(800, 230, 1020, 260);
    wallRect(330, 40, 780, 70);
    wallRect(330, 70, 360, 320);
    wallRect(750, 70, 780, 320);
    wallRect(330, 290, 780, 320);
    wallRect(80, 50, 100, 810);
    wallRect(1000, 50, 1020, 810);
    wallRect(80, 50, 1020, 70);
    wallRect(80, 800, 1020, 820);
    wallRect(940, 100, 980, 320);
    wallRect(780, 100, 980, 140);
    // Keep both the bathroom and intake free of user-built soil.
    for (int r = 0; r < cell_rows; ++r)
        for (int c = 0; c < cell_cols; ++c)
        {
            float x = (c + 0.5f) * cell_dim, y = (r + 0.5f) * cell_dim;
            if ((x >= 360 && x < 750 && y >= 70 && y < 290) ||
                (x >= 850 && x < 940 && y >= 270 && y < 330))
            {
                protected_air[r][c] = 1;
                terrain_solid[r][c] = 0;
            }
        }
    for (int r = 0; r < cell_rows; ++r)
        fixed_wall[r][0] = fixed_wall[r][cell_cols - 1] = 1;
    for (int c = 0; c < cell_cols; ++c)
        fixed_wall[0][c] = fixed_wall[cell_rows - 1][c] = 1;
}
void Fluid::initializeParticles()
{
    // Original HW3 reservoir packing and overflow fallback.
    const float x0 = 120.0f, x1 = 690.0f, y0 = 535.0f;
    const float dx = 2.0f * particle_radius;
    const float dy = std::sqrt(3.0f) * particle_radius;
    float x = x0, y = y0;
    bool stagger = false;
    for (int i = 0; i < num_particles; ++i)
    {
        if (x > x1)
        {
            y += dy;
            stagger = !stagger;
            x = x0 + (stagger ? particle_radius : 0.0f);
        }
        if (y > 755.0f)
        {
            y = y0 + std::fmod(static_cast<float>(i) * 0.37f, 180.0f);
            x = x0 + std::fmod(static_cast<float>(i) * dx * 1.73f, x1 - x0);
        }
        particle_pos[i] = Eigen::Vector2f(x, y);
        particle_vel[i].setZero();
        particle_piped[i] = 0;
        x += dx;
    }
}
void Fluid::resetScene()
{
    collected_particles = emitted_particles = 0;
    scene_time = 0;
    pipe_queue.clear();
    num_particles = initial_particle_count;
    particle_rest_density = 0;
    initializeTerrain();
    initializeParticles();
    for (int r = 0; r < cell_rows; ++r)
        for (int c = 0; c < cell_cols; ++c)
        {
            cell_velocities[r][c].setZero();
            prev_cell_velocities[r][c].setZero();
            cell_densities[r][c] = 0;
        }
    applyTerrainTypes();
}
void Fluid::setTerrainCell(int row, int col, bool solid)
{
    if (row <= 0 || row >= cell_rows - 1 || col <= 0 || col >= cell_cols - 1) return;
    if (fixed_wall[row][col] || protected_air[row][col]) return;
    float x = (col + 0.5f) * cell_dim, y = (row + 0.5f) * cell_dim;
    if (x < 100 || x >= 1000 || y < 70 || y >= 800) return;
    terrain_solid[row][col] = solid ? 1 : 0;
}
void Fluid::carveRect(float x0, float y0, float x1, float y1)
{
    int c0 = std::max(1, static_cast<int>(std::floor(x0 / cell_dim)));
    int c1 = std::min(cell_cols - 2, static_cast<int>(std::floor(x1 / cell_dim)));
    int r0 = std::max(1, static_cast<int>(std::floor(y0 / cell_dim)));
    int r1 = std::min(cell_rows - 2, static_cast<int>(std::floor(y1 / cell_dim)));

    for (int r = r0; r <= r1; ++r)
        for (int c = c0; c <= c1; ++c)
            terrain_solid[r][c] = 0;
}

void Fluid::carveCircle(float cx, float cy, float rad)
{
    float rad2 = rad * rad;
    int c0 = std::max(1, static_cast<int>((cx - rad) / cell_dim));
    int c1 = std::min(cell_cols - 2, static_cast<int>((cx + rad) / cell_dim));
    int r0 = std::max(1, static_cast<int>((cy - rad) / cell_dim));
    int r1 = std::min(cell_rows - 2, static_cast<int>((cy + rad) / cell_dim));

    for (int r = r0; r <= r1; ++r)
    {
        for (int c = c0; c <= c1; ++c)
        {
            float x = (c + 0.5f) * cell_dim;
            float y = (r + 0.5f) * cell_dim;
            float dx = x - cx, dy = y - cy;
            if (dx * dx + dy * dy <= rad2)
                terrain_solid[r][c] = 0;
        }
    }
}

void Fluid::editTerrain(float mouse_x, float mouse_y, float brush_radius)
{
    float wx = mouse_x;
    float wy = viewport_h - mouse_y;
    float r2 = brush_radius * brush_radius;

    int c0 = std::max(1, static_cast<int>((wx - brush_radius) / cell_dim));
    int c1 = std::min(cell_cols - 2, static_cast<int>((wx + brush_radius) / cell_dim));
    int r0 = std::max(1, static_cast<int>((wy - brush_radius) / cell_dim));
    int r1 = std::min(cell_rows - 2, static_cast<int>((wy + brush_radius) / cell_dim));

    for (int r = r0; r <= r1; ++r)
    {
        for (int c = c0; c <= c1; ++c)
        {
            float cx = (c + 0.5f) * cell_dim;
            float cy = (r + 0.5f) * cell_dim;
            float dx = cx - wx, dy = cy - wy;
            if (dx * dx + dy * dy <= r2)
                setTerrainCell(r, c, false);
        }
    }

    applyTerrainTypes();
}
bool Fluid::isTerrainSolid(int row, int col) const
{
    if (row < 0 || row >= cell_rows || col < 0 || col >= cell_cols) return true;
    return terrain_solid[row][col] || fixed_wall[row][col];
}
void Fluid::resolveParticleTerrainCollision(Eigen::Vector2f& pos, Eigen::Vector2f& vel)
{
    const float r = particle_radius;

    for (int pass = 0; pass < 4; ++pass)
    {
        int c0 = static_cast<int>(std::floor((pos.x() - r) / cell_dim));
        int c1 = static_cast<int>(std::floor((pos.x() + r) / cell_dim));
        int r0 = static_cast<int>(std::floor((pos.y() - r) / cell_dim));
        int r1 = static_cast<int>(std::floor((pos.y() + r) / cell_dim));

        float best_pen = 0.0f;
        Eigen::Vector2f best_n(0.0f, 1.0f);
        bool collided = false;

        for (int rr = r0; rr <= r1; ++rr)
        {
            for (int cc = c0; cc <= c1; ++cc)
            {
                if (!isTerrainSolid(rr, cc))
                    continue;

                float x0 = cc * cell_dim;
                float y0 = rr * cell_dim;
                float x1 = x0 + cell_dim;
                float y1 = y0 + cell_dim;

                float qx = clampf(pos.x(), x0, x1);
                float qy = clampf(pos.y(), y0, y1);
                Eigen::Vector2f d = pos - Eigen::Vector2f(qx, qy);
                float d2 = d.squaredNorm();

                float pen = 0.0f;
                Eigen::Vector2f n;

                if (d2 > EPS)
                {
                    float dist = std::sqrt(d2);
                    if (dist >= r)
                        continue;
                    n = d / dist;
                    pen = r - dist;
                }
                else
                {
                    // Center is inside a solid cell: push toward nearest face.
                    float dl = std::fabs(pos.x() - x0);
                    float dr = std::fabs(x1 - pos.x());
                    float db = std::fabs(pos.y() - y0);
                    float dtp = std::fabs(y1 - pos.y());

                    float m = dl;
                    n = Eigen::Vector2f(-1.0f, 0.0f);
                    pen = r + dl;
                    if (dr < m) { m = dr; n = Eigen::Vector2f(1.0f, 0.0f); pen = r + dr; }
                    if (db < m) { m = db; n = Eigen::Vector2f(0.0f, -1.0f); pen = r + db; }
                    if (dtp < m){ n = Eigen::Vector2f(0.0f, 1.0f); pen = r + dtp; }
                }

                if (pen > best_pen)
                {
                    best_pen = pen;
                    best_n = n;
                    collided = true;
                }
            }
        }

        if (!collided)
            break;

        pos += (best_pen + 0.02f) * best_n;
        float vn = vel.dot(best_n);
        if (vn < 0.0f)
            vel -= vn * best_n;
    }

    pos.x() = clampf(pos.x(), particle_radius + cell_dim, viewport_w - particle_radius - cell_dim);
    pos.y() = clampf(pos.y(), particle_radius + cell_dim, viewport_h - particle_radius - cell_dim);
}

void Fluid::removeParticle(int i)
{
    --num_particles;
    particle_pos[i] = particle_pos[num_particles];
    particle_vel[i] = particle_vel[num_particles];
    particle_piped[i] = particle_piped[num_particles];
}
void Fluid::updatePipe()
{
    for (float& remaining : pipe_queue) remaining -= dt;
    int released = 0;
    for (size_t i = 0; i < pipe_queue.size() && released < 4;)
    {
        if (pipe_queue[i] > 0) { ++i; continue; }
        const int lane = emitted_particles++ % 7;
        particle_pos[num_particles] = Eigen::Vector2f(706, 222 + lane * 2.0f * particle_radius);
        particle_vel[num_particles] = Eigen::Vector2f(-120, -32);
        particle_piped[num_particles++] = 1;
        pipe_queue.erase(pipe_queue.begin() + i);
        ++released;
    }
    for (int i = 0; i < num_particles;)
    {
        const auto& p = particle_pos[i];
        if (!particle_piped[i] && p.x() >= 882 && p.x() < 942 && p.y() >= 270 && p.y() <= 334)
        {
            pipe_queue.push_back(0.65f);
            removeParticle(i);
        }
        else ++i;
    }
}
void Fluid::collectDrainParticles()
{
    for (int i = 0; i < num_particles;)
    {
        const auto& p = particle_pos[i];
        // Score only water that actually falls into the visible tub basin.
        // The particle must also have passed through the pipe first.
        if (particle_piped[i] && p.x() >= 400 && p.x() <= 655 &&
            p.y() >= 75 && p.y() <= 155)
        {
            removeParticle(i);
            ++collected_particles;
        }
        else ++i;
    }
}
float Fluid::getCollectedRatio() const
{
    return initial_particle_count > 0
        ? static_cast<float>(collected_particles) / static_cast<float>(initial_particle_count)
        : 0.0f;
}

float Fluid::getGoalProgress() const
{
    // Kept as a compatibility accessor for UI/tests. Progress now means the
    // true fraction of the initial water that has been collected.
    return clampf(getCollectedRatio(), 0.0f, 1.0f);
}

bool Fluid::isLevelCleared() const
{
    return getCollectedRatio() >= getClearThreshold();
}

int Fluid::getStarRating() const
{
    const float ratio = getCollectedRatio();
    if (ratio >= 0.95f) return 3;
    if (ratio >= 0.85f) return 2;
    if (ratio >= 0.70f) return 1;
    return 0;
}

void Fluid::update()
{
    scene_time += dt;
    integrate();
    particleRelaxation();
    handleCollisions();
    transferVelocities(true);
    updateDensity();
    solveIncompressibility();
    transferVelocities(false);
    updatePipe();
    collectDrainParticles();
    applyTerrainTypes();
}
