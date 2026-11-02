#include "fluid.h"
#include "simulation_defaults.h"
#include <iostream>
#include <cmath>

namespace D = SimulationDefaults;

bool Fluid::selfTest()
{
    auto require = [](bool ok, const char* message) {
        if (!ok) std::cerr << "FAIL: " << message << '\n';
        return ok;
    };
    auto originalFluid = [](int count) {
        return Fluid(count, D::ParticleRadius, D::FlipRatio, D::CellSize, D::RelaxationCellSize,
                     D::Iterations, D::DomainWidth, D::DomainHeight, D::TimeStep,
                     D::Gravity, D::Stiffness, D::DensityCorrection, D::OverRelaxation);
    };
    Fluid f = originalFluid(84);
    f.collected_particles = 29;
    if (!require(std::fabs(f.getGoalProgress() - (29.0f / 84)) < 1e-6f,
                 "goal progress shows the true collected fraction")) return false;
    for (int count : {59, 76, 84}) {
        f.collected_particles = count;
        if (!require(std::fabs(f.getGoalProgress() - (float(count) / 84.0f)) < 1e-6f,
                     "goal bar remains proportional above the clear threshold")) return false;
    }
    f.resetScene();
    const auto walls = f.fixed_wall;
    f.editTerrain(615, 475, 65);
    if (!require(walls == f.fixed_wall && f.isTerrainSolid(42, 61), "fixed wall survives digging")) return false;
    f.resetScene();
    for (int i = 0; i < f.num_particles; ++i) {
        f.particle_pos[i] = Eigen::Vector2f(887 + (i % 7) * 7, 276 + (i / 7) * 4);
        f.particle_vel[i].setZero();
    }
    f.updatePipe();
    if (!require(f.num_particles == 0 && f.pipe_queue.size() == 84 && f.collected_particles == 0,
                 "intake transfers water into pipe without counting it as collected")) return false;
    for (int step = 0; step < 450; ++step) {
        f.update();
        if (!require(f.num_particles + f.getPipeParticleCount() + f.collected_particles == 84,
                     "water count conserved across pipe and bath")) return false;
        for (int i = 0; i < f.num_particles; ++i)
            if (!require(f.particle_pos[i].allFinite() && f.particle_vel[i].allFinite(), "finite particle state")) return false;
    }
    std::cout << "Pipe delivery: " << f.collected_particles << " / 84\n";
    if (!require(f.isLevelCleared() && f.getPipeParticleCount() == 0, "nozzle delivers enough simulated water to clear")) return false;
    f.resetScene();
    if (!require(f.num_particles == 84 && f.collected_particles == 0 && f.pipe_queue.empty(), "reset restores water and empties pipe")) return false;
    f.particle_pos[0] = Eigen::Vector2f(500, 100);
    f.collectDrainParticles();
    if (!require(f.collected_particles == 0, "water cannot bypass the pipe to score")) return false;
    // Even piped water must be inside the visible tub, not merely somewhere
    // in the larger bathroom cavity.
    f.particle_piped[0] = 1;
    f.particle_pos[0] = Eigen::Vector2f(700, 100);
    f.collectDrainParticles();
    if (!require(f.collected_particles == 0, "piped water outside the tub does not score")) return false;
    f.particle_pos[0] = Eigen::Vector2f(500, 100);
    f.collectDrainParticles();
    if (!require(f.collected_particles == 1, "piped water inside the tub scores")) return false;

    // Exercise a complete player-dug route with the real PIC/FLIP pipeline.
    Fluid level = originalFluid(D::ParticleCount);
    if (!require(level.cell_cols == 160 && level.cell_rows == 90 && level.num_particles == 2600 &&
                 level.particle_radius == 4.0f && level.dt == 0.035f && level.gravity == 18.0f,
                 "original HW3 particle settings, grid and time step")) return false;
    if (!require((level.particle_pos[0] - Eigen::Vector2f(120, 535)).norm() < 1e-5f &&
                 (level.particle_pos[71] - Eigen::Vector2f(688, 535)).norm() < 1e-5f &&
                 (level.particle_pos[72] - Eigen::Vector2f(124, 535 + std::sqrt(3.0f) * 4)).norm() < 1e-5f,
                 "original HW3 reservoir particle packing")) return false;
    for (int i = 0; i < level.num_particles; ++i) {
        const auto& q = level.particle_pos[i];
        if (!require(!level.isTerrainSolid(int(q.y() / 10), int(q.x() / 10)),
                     "original water volume fits the redesigned reservoir")) return false;
    }
    const Eigen::Vector2f route[] = {{680, 535}, {710, 495}, {735, 465}, {850, 455}, {900, 400}, {900, 335}, {904, 300}};
    for (int segment = 1; segment < 7; ++segment)
        for (int k = 0; k <= 30; ++k) {
            Eigen::Vector2f q = route[segment - 1] + (route[segment] - route[segment - 1]) * (k / 30.0f);
            level.editTerrain(q.x(), 900 - q.y(), 38);
        }
    for (int step = 0; step < 4000 && !level.isLevelCleared(); ++step) {
        level.update();
        if (!require(level.num_particles + level.getPipeParticleCount() + level.collected_particles == 2600,
                     "full level water conservation")) return false;
        for (int i = 0; i < level.num_particles; ++i)
            if (!require(level.particle_pos[i].allFinite() && level.particle_vel[i].allFinite(), "stable full level simulation")) return false;
    }
    std::cout << "Dug route delivery: " << level.collected_particles << " / 2600\n";
    if (!require(level.isLevelCleared(), "dug route can complete the level")) return false;
    std::cout << "PASS: true collection progress, original fluid settings/packing, walls, pipe, tub scoring, conservation, reset, full level\n";
    return true;
}
