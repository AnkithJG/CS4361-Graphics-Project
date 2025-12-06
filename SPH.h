#ifndef SPH_H
#define SPH_H

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

const float DT = 0.008f;
const float H = 0.06f;
const float H2 = H * H;
const float MASS = 0.0002f;
const float RHO0 = 1000.0f;
const float K = 2000.0f;
const float MU = 0.8f;
const float G = 15.0f;
const float WALL_DAMPING = -0.3f;

const float POLY6_COEFF = 315.0f / (64.0f * glm::pi<float>() * glm::pow(H, 9));
const float SPIKY_GRAD_COEFF = -45.0f / (glm::pi<float>() * glm::pow(H, 6));
const float VISC_LAPL_COEFF = 45.0f / (glm::pi<float>() * glm::pow(H, 6));

struct Particle
{
    glm::vec3 pos;
    glm::vec3 vel;
    glm::vec3 acc;
    float density;
    float pressure;
    int cell_id;
};

class SPHSolver
{
public:
    SPHSolver(int num_particles);
    ~SPHSolver();

    void update();
    const std::vector<Particle> &getParticles() const { return particles; }

private:
    std::vector<Particle> particles;

    std::vector<int> grid_map;
    std::vector<int> start_index;
    int grid_size_x, grid_size_y, grid_size_z;
    float cell_size;

    void init_particles(int num_particles);

    int get_cell_id(const glm::vec3 &pos);
    void update_spatial_hash();
    void find_neighbors(int particle_index, std::vector<int> &neighbors);

    void compute_density_pressure();
    void compute_forces();
    void integrate();
    void handle_boundary();

    float W_poly6(float r2);
    glm::vec3 W_spiky_grad(const glm::vec3 &r, float r_len);
    float W_visc_lapl(float r_len);
};

#endif
