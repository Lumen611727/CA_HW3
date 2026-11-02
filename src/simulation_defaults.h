#pragma once

// Defaults from the original HW3 prototype. Scene and UI changes must not tune
// these values or change the one-simulation-step-per-rendered-frame cadence.
namespace SimulationDefaults
{
constexpr int ParticleCount = 2600;
constexpr float ParticleRadius = 4.0f;
constexpr float FlipRatio = 0.92f;
constexpr float CellSize = 10.0f;
constexpr float RelaxationCellSize = 10.0f;
constexpr int Iterations = 4;
constexpr float DomainWidth = 1600.0f;
constexpr float DomainHeight = 900.0f;
constexpr float TimeStep = 0.035f;
constexpr float Gravity = 18.0f;
constexpr float Stiffness = 1.0f;
constexpr bool DensityCorrection = true;
constexpr float OverRelaxation = 1.85f;
}
