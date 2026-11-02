#pragma once
#include <vector>
#include <Eigen/Dense>
enum CellType { AIR, FLUID, SOLID };
class Fluid
{
public:
    Fluid(int num_particles, float radius, float flip_ratio, float cell_dim,
          float relaxation_cell_dim, int iterations, float viewport_w,
          float viewport_h, float dt, float gravity, float stiffness,
          bool density_correction, float over_relaxation);
    void update();
    void drawScene(int render_option) const;
    void editTerrain(float mouse_x, float mouse_y, float brush_radius);
    void resetScene();
    static bool selfTest();
    void setFlipRatio(float v) { flip_ratio = v; }
    void setStiffnessCoeff(float v) { stiffness_coefficient = v; }
    void setDt(float v) { dt = v; }
    void setGravity(float v) { gravity = v; }
    void setDensityCorrection(bool v) { density_correction = v; }
    void setOverRelaxation(float v) { over_relaxation = v; }
    void setIterations(int v) { iterations = v; }
    int getCollectedParticleCount() const { return collected_particles; }
    int getInitialParticleCount() const { return initial_particle_count; }
    int getActiveParticleCount() const { return num_particles; }
    int getPipeParticleCount() const { return static_cast<int>(pipe_queue.size()); }
    float getCollectedRatio() const;
    float getGoalProgress() const;
    float getClearThreshold() const { return 0.70f; }
    bool isLevelCleared() const;
    int getStarRating() const;
private:
    int iterations;
    float dt, gravity, viewport_w, viewport_h;
    float flip_ratio, stiffness_coefficient, over_relaxation;
    bool density_correction;
    std::vector<Eigen::Vector2f> particle_pos, particle_vel;
    std::vector<unsigned char> particle_piped;
    float particle_radius;
    int num_particles, initial_particle_count, collected_particles;
    float particle_rest_density;
    float relaxation_cell_dim;
    int relaxation_cell_rows, relaxation_cell_cols;
    std::vector<std::vector<std::vector<int>>> relaxation_cell_particle_ids;
    int cell_rows, cell_cols;
    float cell_dim;
    std::vector<std::vector<CellType>> cell_types;
    std::vector<std::vector<Eigen::Vector2f>> cell_velocities, prev_cell_velocities;
    std::vector<std::vector<float>> cell_densities;
    std::vector<std::vector<unsigned char>> terrain_solid, fixed_wall, protected_air;
    std::vector<float> pipe_queue;
    float scene_time = 0;
    int emitted_particles = 0;
    void integrate();
    void particleRelaxation();
    void handleCollisions();
    void updateDensity();
    void transferVelocities(bool to_cell);
    void solveIncompressibility();
    void updatePipe();
    void collectDrainParticles();
    void removeParticle(int index);
    void initializeTerrain();
    void initializeParticles();
    void applyTerrainTypes();
    void setTerrainCell(int row, int col, bool solid);
    void carveRect(float x0, float y0, float x1, float y1);
    void carveCircle(float cx, float cy, float radius);
    void wallRect(float x0, float y0, float x1, float y1);
    bool isTerrainSolid(int row, int col) const;
    bool isFaceVelocityValid(int row, int col, int component) const;
    void resolveParticleTerrainCollision(Eigen::Vector2f& pos, Eigen::Vector2f& vel);
};
