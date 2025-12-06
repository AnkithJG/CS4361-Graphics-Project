// SPH.h

#ifndef SPH_H
#define SPH_H

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

// --- SPH Constants (Physics Parameters) ---
const float DT = 0.008f;          // Time step
const float H = 0.06f;            // Smoothing length (radius of influence)
const float H2 = H * H;           // H squared
const float MASS = 0.0002f;       // Particle mass
const float RHO0 = 1000.0f;       // Rest density of water (1000 kg/m^3)
const float K = 2000.0f;          // Gas constant (for pressure calculation)
const float MU = 0.8f;            // Viscosity constant
const float G = 15.0f;            // Gravity (m/s^2)
const float WALL_DAMPING = -0.3f; // Damping on collision

// Precomputed kernel coefficients
const float POLY6_COEFF = 315.0f / (64.0f * glm::pi<float>() * glm::pow(H, 9));
const float SPIKY_GRAD_COEFF = -45.0f / (glm::pi<float>() * glm::pow(H, 6));
const float VISC_LAPL_COEFF = 45.0f / (glm::pi<float>() * glm::pow(H, 6));

// --- Particle Structure ---
struct Particle
{
    glm::vec3 pos;  // Position (P)
    glm::vec3 vel;  // Velocity (V)
    glm::vec3 acc;  // Acceleration (A)
    float density;  // Density (Rho)
    float pressure; // Pressure (P)

    // For neighbor search via Spatial Hashing
    int cell_id;
};

// --- SPH Solver Class ---
class SPHSolver
{
public:
    SPHSolver(int num_particles);
    ~SPHSolver();

    void update();
    const std::vector<Particle> &getParticles() const { return particles; }

private:
    std::vector<Particle> particles;

    // --- Spatial Hashing Structures ---
    std::vector<int> grid_map;
    std::vector<int> start_index;
    int grid_size_x, grid_size_y, grid_size_z;
    float cell_size;

    void init_particles(int num_particles);

    // Spatial Hashing methods
    int get_cell_id(const glm::vec3 &pos);
    void update_spatial_hash();
    void find_neighbors(int particle_index, std::vector<int> &neighbors);

    // Core SPH methods
    void compute_density_pressure();
    void compute_forces();
    void integrate();
    void handle_boundary();

    // SPH Kernels
    float W_poly6(float r2);
    glm::vec3 W_spiky_grad(const glm::vec3 &r, float r_len);
    float W_visc_lapl(float r_len);
};

#endif