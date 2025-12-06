#include "SPH.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <chrono>
#include <omp.h>

using namespace glm;

// set up the spatial hash grid and initialize particles
SPHSolver::SPHSolver(int num_particles)
{
    // grid cell size is 2*H so each particle only needs to check adjacent cells
    cell_size = H * 2.0f;
    grid_size_x = grid_size_y = grid_size_z = 10; // 10x10x10 grid
    int total_cells = grid_size_x * grid_size_y * grid_size_z;
    grid_map.resize(total_cells, -1);
    start_index.resize(total_cells + 1, -1);

    init_particles(num_particles);
}

SPHSolver::~SPHSolver() {}

void SPHSolver::init_particles(int num_particles)
{
    particles.reserve(num_particles);

    // spawn particles in a cube with regular spacing
    float spacing = H * 0.55f; // slightly less than kernel radius
    float block_size = 0.8f;   // cube size

    // starting position of the cube
    float start_x = 0.6f;
    float start_y = 0.05f;
    float start_z = 0.6f;

    float end_x = start_x + block_size;
    float end_y = start_y + block_size;
    float end_z = start_z + block_size;

    // calculate center so we can make particles "explode" outward
    glm::vec3 center(start_x + block_size / 2.0f, start_y + block_size / 2.0f, start_z + block_size / 2.0f);

    // create particles in 3d grid
    for (float y = start_y; y < end_y && particles.size() < num_particles; y += spacing)
    {
        for (float z = start_z; z < end_z && particles.size() < num_particles; z += spacing)
        {
            for (float x = start_x; x < end_x && particles.size() < num_particles; x += spacing)
            {
                Particle p;
                p.pos = vec3(x, y, z);

                // give initial outward velocity for cool explosion effect
                glm::vec3 dir = glm::normalize(p.pos - center);
                p.vel = dir * 1.5f;

                p.acc = vec3(0.0f);
                p.density = RHO0; // assume normal density to start
                p.pressure = 0.0f;
                particles.push_back(p);
            }
        }
    }

    std::cout << "Initialized " << particles.size()
              << " particles - exploding outward!" << std::endl;
}

// polynomial kernel - used for density calculations
// particles influence each other smoothly based on distance
float SPHSolver::W_poly6(float r2)
{
    if (r2 < 0.0f || r2 > H2) // outside kernel radius = no influence
        return 0.0f;
    float h2_minus_r2 = H2 - r2;
    // smooth cubic dropoff
    return POLY6_COEFF * h2_minus_r2 * h2_minus_r2 * h2_minus_r2;
}

// gradient of spiky kernel - used for pressure forces
// creates sharper pressure waves than the poly6 kernel
glm::vec3 SPHSolver::W_spiky_grad(const glm::vec3 &r, float r_len)
{
    if (r_len == 0.0f || r_len > H)
        return vec3(0.0f);
    float h_minus_r = H - r_len;
    // returns direction and magnitude of pressure force
    return r * (SPIKY_GRAD_COEFF * h_minus_r * h_minus_r / r_len);
}

// laplacian of viscosity kernel - smooths out particle velocities
// makes particles try to move together
float SPHSolver::W_visc_lapl(float r_len)
{
    if (r_len > H)
        return 0.0f;
    return VISC_LAPL_COEFF * (H - r_len);
}

// convert 3d position to grid cell index
int SPHSolver::get_cell_id(const glm::vec3 &pos)
{
    // divide space into grid
    int ix = (int)std::floor(pos.x / cell_size);
    int iy = (int)std::floor(pos.y / cell_size);
    int iz = (int)std::floor(pos.z / cell_size);

    // clamp to grid bounds
    ix = glm::clamp(ix, 0, grid_size_x - 1);
    iy = glm::clamp(iy, 0, grid_size_y - 1);
    iz = glm::clamp(iz, 0, grid_size_z - 1);

    // convert 3d coordinates to 1d index
    return ix + grid_size_x * (iy + grid_size_y * iz);
}

// reorganize particles so particles in same cell are adjacent in memory
// makes neighbor lookup way faster (cache friendly)
void SPHSolver::update_spatial_hash()
{
    // create list of (cell_id, particle_id) pairs
    std::vector<std::pair<int, int>> sorted_particles;
    sorted_particles.reserve(particles.size());
    for (int i = 0; i < particles.size(); ++i)
    {
        particles[i].cell_id = get_cell_id(particles[i].pos);
        sorted_particles.push_back({particles[i].cell_id, i});
    }

    // sort by cell id so particles from same cell are together
    std::sort(sorted_particles.begin(), sorted_particles.end());

    // reorder particle array based on sorted order
    std::vector<Particle> temp_particles = particles;
    for (size_t i = 0; i < sorted_particles.size(); ++i)
    {
        particles[i] = temp_particles[sorted_particles[i].second];
    }

    // build start index: tells us where each cell's particles begin
    std::fill(start_index.begin(), start_index.end(), -1);
    for (size_t i = 0; i < particles.size(); ++i)
    {
        int cell_id = particles[i].cell_id;
        if (i == 0 || particles[i - 1].cell_id != cell_id)
        {
            start_index[cell_id] = i; // first particle in this cell
        }
    }
}

// find nearby particles using spatial hash
void SPHSolver::find_neighbors(int particle_index, std::vector<int> &neighbors)
{
    neighbors.clear();
    const Particle &pi = particles[particle_index];

    const int MAX_NEIGHBORS = 30; // limit so computation doesn't explode

    // get this particle's grid coordinates
    int cell_x = pi.cell_id % grid_size_x;
    int cell_y = (pi.cell_id / grid_size_x) % grid_size_y;
    int cell_z = pi.cell_id / (grid_size_x * grid_size_y);

    // check the 27 surrounding cells (3x3x3 cube)
    for (int dx = -1; dx <= 1; ++dx)
    {
        for (int dy = -1; dy <= 1; ++dy)
        {
            for (int dz = -1; dz <= 1; ++dz)
            {
                int nx = cell_x + dx;
                int ny = cell_y + dy;
                int nz = cell_z + dz;

                // check bounds
                if (nx >= 0 && nx < grid_size_x &&
                    ny >= 0 && ny < grid_size_y &&
                    nz >= 0 && nz < grid_size_z)
                {
                    // convert back to 1d cell index
                    int neighbor_cell_id = nx + grid_size_x * (ny + grid_size_y * nz);
                    int start = start_index[neighbor_cell_id];

                    if (start != -1)
                    {
                        // iterate through all particles in this cell
                        for (size_t j = start; j < particles.size(); ++j)
                        {
                            if (particles[j].cell_id != neighbor_cell_id)
                                break; // moved to next cell, stop

                            // check if actually within kernel radius
                            vec3 r = pi.pos - particles[j].pos;
                            float r2 = dot(r, r);
                            if (r2 < H2)
                            {
                                neighbors.push_back(j);

                                if (neighbors.size() >= MAX_NEIGHBORS)
                                    return;
                            }
                        }
                    }
                }
            }
        }
    }
}

void SPHSolver::compute_density_pressure()
{
#pragma omp parallel for // parallelize over particles
    for (int i = 0; i < particles.size(); ++i)
    {
        std::vector<int> neighbors;
        neighbors.reserve(100);

        // density = sum of masses of nearby particles
        particles[i].density = MASS * W_poly6(0.0f); // self contribution

        find_neighbors(i, neighbors);
        for (int j : neighbors)
        {
            vec3 r = particles[i].pos - particles[j].pos;
            float r2 = dot(r, r);
            particles[i].density += MASS * W_poly6(r2); // add neighbor contributions
        }

        // don't let density go to zero (avoid division by zero)
        if (particles[i].density < RHO0 * 0.01f)
        {
            particles[i].density = RHO0 * 0.01f;
        }

        // pressure from equation of state - high exponent makes it incompressible
        particles[i].pressure = K * (glm::pow(particles[i].density / RHO0, 7.0f) - 1.0f);
        if (particles[i].pressure < 0.0f) // pressure can't be negative
            particles[i].pressure = 0.0f;
    }
}

// calculate forces: pressure, viscosity, and gravity
void SPHSolver::compute_forces()
{
#pragma omp parallel for
    for (int i = 0; i < particles.size(); ++i)
    {
        std::vector<int> neighbors;
        neighbors.reserve(100);

        vec3 F_pressure(0.0f);  // pushes particles apart
        vec3 F_viscosity(0.0f); // smooths velocities

        find_neighbors(i, neighbors);
        for (int j : neighbors)
        {
            if (i == j)
                continue; // don't self-interact

            vec3 r = particles[i].pos - particles[j].pos;
            float r_len = length(r);

            // pressure pushes away from crowded areas
            // uses average pressure of both particles
            float pressure_term = (particles[i].pressure + particles[j].pressure) / (2.0f * particles[j].density);
            F_pressure -= MASS * pressure_term * W_spiky_grad(r, r_len);

            // viscosity: neighbors' velocity pulls this one toward them
            // makes fluid "stick together"
            vec3 vel_diff = particles[j].vel - particles[i].vel;
            F_viscosity += MASS * (vel_diff / particles[j].density) * MU * W_visc_lapl(r_len);
        }

        // gravity pulls downward
        vec3 F_gravity = MASS * vec3(0.0f, -G, 0.0f);
        // acceleration = force / mass
        particles[i].acc = (F_pressure + F_viscosity + F_gravity) / MASS;
    }
}

// update velocity and position using forces (forward euler integration)
void SPHSolver::integrate()
{
    const float MAX_VEL = 10.0f; // cap speed so simulation doesn't blow up

    for (auto &p : particles)
    {
        // v = v + a*dt
        p.vel += p.acc * DT;

        // clamp velocity magnitude
        float vel_len = glm::length(p.vel);
        if (vel_len > MAX_VEL)
        {
            p.vel = (p.vel / vel_len) * MAX_VEL;
        }

        // check for NaN (sometimes happens with bad parameters)
        if (std::isnan(p.vel.x) || std::isnan(p.vel.y) || std::isnan(p.vel.z))
        {
            p.vel = glm::vec3(0.0f);
        }
        if (std::isnan(p.pos.x) || std::isnan(p.pos.y) || std::isnan(p.pos.z))
        {
            p.pos = glm::vec3(1.0f); // reset to center
        }

        // x = x + v*dt
        p.pos += p.vel * DT;
    }
}

// bounce particles off walls
void SPHSolver::handle_boundary()
{
    float tank_max = 1.5f;      // container size
    float boundary_min = 0.01f; // wall thickness

    for (auto &p : particles)
    {
        // x-axis walls
        if (p.pos.x < boundary_min)
        {
            p.pos.x = boundary_min;
            p.vel.x *= WALL_DAMPING; // -0.3 so it bounces with energy loss
        }
        else if (p.pos.x > tank_max - boundary_min)
        {
            p.pos.x = tank_max - boundary_min;
            p.vel.x *= WALL_DAMPING;
        }

        // y-axis (gravity direction)
        if (p.pos.y < boundary_min)
        {
            p.pos.y = boundary_min;
            p.vel.y *= WALL_DAMPING;
        }
        else if (p.pos.y > tank_max - boundary_min)
        {
            p.pos.y = tank_max - boundary_min;
            p.vel.y *= WALL_DAMPING;
        }

        // z-axis walls
        if (p.pos.z < boundary_min)
        {
            p.pos.z = boundary_min;
            p.vel.z *= WALL_DAMPING;
        }
        else if (p.pos.z > tank_max - boundary_min)
        {
            p.pos.z = tank_max - boundary_min;
            p.vel.z *= WALL_DAMPING;
        }
    }
}

// main update function - runs one timestep of simulation
void SPHSolver::update()
{
    static int frame_count = 0;
    auto start = std::chrono::high_resolution_clock::now();

    // step 1: rebuild acceleration structure for neighbor lookup
    update_spatial_hash();

    // step 2: calculate how compressed particles are
    compute_density_pressure();

    // step 3: calculate forces from pressure, viscosity, gravity
    compute_forces();

    // step 4: update velocities and positions
    integrate();

    // step 5: enforce walls (don't let particles leave container)
    handle_boundary();

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    // print stats every 60 frames
    if (frame_count % 60 == 0)
    {
        std::cout << "Frame " << frame_count
                  << " | Update time: " << duration.count() << "ms"
                  << " | Particle 0 pos: (" << particles[0].pos.x << ", "
                  << particles[0].pos.y << ", " << particles[0].pos.z << ")"
                  << " | Total particles: " << particles.size() << std::endl;
    }
    frame_count++;
}