#ifndef SPH_H
#define SPH_H

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

// simulation params - tweak these to change how the fluid behaves
const float DT = 0.008f;          // timestep - smaller = more accurate but slower
const float H = 0.06f;            // kernel radius - how far particles "feel" each other
const float H2 = H * H;           // squared version (avoid recalculating)
const float MASS = 0.0002f;       // mass per particle
const float RHO0 = 1000.0f;       // target density (water reference)
const float K = 2000.0f;          // stiffness - makes fluid resist compression
const float MU = 0.8f;            // viscosity - higher = more sticky/thick
const float G = 15.0f;            // gravity strength
const float WALL_DAMPING = -0.3f; // energy loss when hitting walls (negative so it bounces back)

// kernel coefficients - used in the math for pressure/viscosity calculations
// don't worry too much about these, they're just normalization constants
const float POLY6_COEFF = 315.0f / (64.0f * glm::pi<float>() * glm::pow(H, 9));
const float SPIKY_GRAD_COEFF = -45.0f / (glm::pi<float>() * glm::pow(H, 6));
const float VISC_LAPL_COEFF = 45.0f / (glm::pi<float>() * glm::pow(H, 6));

// represents a single particle in the simulation
struct Particle
{
    glm::vec3 pos;  // where it is
    glm::vec3 vel;  // how fast it's moving
    glm::vec3 acc;  // acceleration from forces
    float density;  // how many neighbors around it (affects pressure)
    float pressure; // pressure force (pushes it away from crowded areas)
    int cell_id;    // which grid cell it's in (for fast neighbor lookup)
};

// main sph solver class - does all the particle simulation stuff
class SPHSolver
{
public:
    SPHSolver(int num_particles);
    ~SPHSolver();

    void update(); // run one timestep of simulation
    const std::vector<Particle> &getParticles() const { return particles; }

private:
    // the actual particle data
    std::vector<Particle> particles;

    // spatial hash grid - speeds up finding neighbors instead of checking all particles
    // basically divides space into a 3d grid so we only check nearby cells
    std::vector<int> grid_map;                 // not really used but keeping it around
    std::vector<int> start_index;              // points to first particle in each grid cell
    int grid_size_x, grid_size_y, grid_size_z; // grid dimensions
    float cell_size;                           // size of each cell (2*H)

    // init stuff
    void init_particles(int num_particles);

    // spatial hash helpers
    int get_cell_id(const glm::vec3 &pos);                                // convert position to grid cell index
    void update_spatial_hash();                                           // reorganize particles by cell for faster lookup
    void find_neighbors(int particle_index, std::vector<int> &neighbors); // get nearby particles

    // sph computation steps
    void compute_density_pressure(); // figure out how compressed each particle is
    void compute_forces();           // calc pressure/viscosity/gravity forces
    void integrate();                // update velocity and position
    void handle_boundary();          // bounce particles off walls

    // kernel functions - these describe how particles influence each other
    // further away = less influence, used for smooth interpolation
    float W_poly6(float r2);
    glm::vec3 W_spiky_grad(const glm::vec3 &r, float r_len);
    float W_visc_lapl(float r_len);
};

#endif