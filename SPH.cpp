// SPH.cpp - Fixed to drop water from top

#include "SPH.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <chrono>
#include <omp.h>

using namespace glm;

// --- Constructor and Initialization ---

SPHSolver::SPHSolver(int num_particles)
{
    // Setup for Spatial Hashing
    cell_size = H * 2.0f;
    grid_size_x = grid_size_y = grid_size_z = 10; // 3D tank size 0-2 (scaled)
    int total_cells = grid_size_x * grid_size_y * grid_size_z;
    grid_map.resize(total_cells, -1);
    start_index.resize(total_cells + 1, -1);

    init_particles(num_particles);
}

SPHSolver::~SPHSolver() {}

void SPHSolver::init_particles(int num_particles)
{
    particles.reserve(num_particles);

    float spacing = H * 0.6f;

    // Drop water from ABOVE the container center
    float block_size = 1.0f; // Compact cube
    float start_x = 0.5f;
    float start_y = 1.2f; // START HIGH - gravity will pull it down
    float start_z = 0.5f;

    float end_x = start_x + block_size;
    float end_y = start_y + block_size;
    float end_z = start_z + block_size;

    for (float y = start_y; y < end_y && particles.size() < num_particles; y += spacing)
    {
        for (float z = start_z; z < end_z && particles.size() < num_particles; z += spacing)
        {
            for (float x = start_x; x < end_x && particles.size() < num_particles; x += spacing)
            {
                Particle p;
                p.pos = vec3(x, y, z);

                // Give particles a small downward initial velocity
                p.vel = vec3(0.0f, -0.5f, 0.0f); // Slight downward push

                p.acc = vec3(0.0f);
                p.density = RHO0;
                p.pressure = 0.0f;
                particles.push_back(p);
            }
        }
    }

    std::cout << "Initialized " << particles.size()
              << " particles at height " << start_y
              << " - dropping!" << std::endl;
}

// --- Kernel Functions ---

float SPHSolver::W_poly6(float r2)
{
    if (r2 < 0.0f || r2 > H2)
        return 0.0f;
    float h2_minus_r2 = H2 - r2;
    return POLY6_COEFF * h2_minus_r2 * h2_minus_r2 * h2_minus_r2;
}

glm::vec3 SPHSolver::W_spiky_grad(const glm::vec3 &r, float r_len)
{
    if (r_len == 0.0f || r_len > H)
        return vec3(0.0f);
    float h_minus_r = H - r_len;
    return r * (SPIKY_GRAD_COEFF * h_minus_r * h_minus_r / r_len);
}

float SPHSolver::W_visc_lapl(float r_len)
{
    if (r_len > H)
        return 0.0f;
    return VISC_LAPL_COEFF * (H - r_len);
}

// --- Spatial Hashing ---

int SPHSolver::get_cell_id(const glm::vec3 &pos)
{
    int ix = (int)std::floor(pos.x / cell_size);
    int iy = (int)std::floor(pos.y / cell_size);
    int iz = (int)std::floor(pos.z / cell_size);

    ix = glm::clamp(ix, 0, grid_size_x - 1);
    iy = glm::clamp(iy, 0, grid_size_y - 1);
    iz = glm::clamp(iz, 0, grid_size_z - 1);

    return ix + grid_size_x * (iy + grid_size_y * iz);
}

void SPHSolver::update_spatial_hash()
{
    std::vector<std::pair<int, int>> sorted_particles;
    sorted_particles.reserve(particles.size());
    for (int i = 0; i < particles.size(); ++i)
    {
        particles[i].cell_id = get_cell_id(particles[i].pos);
        sorted_particles.push_back({particles[i].cell_id, i});
    }

    std::sort(sorted_particles.begin(), sorted_particles.end());

    std::vector<Particle> temp_particles = particles;
    for (size_t i = 0; i < sorted_particles.size(); ++i)
    {
        particles[i] = temp_particles[sorted_particles[i].second];
    }

    std::fill(start_index.begin(), start_index.end(), -1);
    for (size_t i = 0; i < particles.size(); ++i)
    {
        int cell_id = particles[i].cell_id;
        if (i == 0 || particles[i - 1].cell_id != cell_id)
        {
            start_index[cell_id] = i;
        }
    }
}

void SPHSolver::find_neighbors(int particle_index, std::vector<int> &neighbors)
{
    neighbors.clear();
    const Particle &pi = particles[particle_index];

    const int MAX_NEIGHBORS = 30;

    int cell_x = pi.cell_id % grid_size_x;
    int cell_y = (pi.cell_id / grid_size_x) % grid_size_y;
    int cell_z = pi.cell_id / (grid_size_x * grid_size_y);

    for (int dx = -1; dx <= 1; ++dx)
    {
        for (int dy = -1; dy <= 1; ++dy)
        {
            for (int dz = -1; dz <= 1; ++dz)
            {
                int nx = cell_x + dx;
                int ny = cell_y + dy;
                int nz = cell_z + dz;

                if (nx >= 0 && nx < grid_size_x &&
                    ny >= 0 && ny < grid_size_y &&
                    nz >= 0 && nz < grid_size_z)
                {
                    int neighbor_cell_id = nx + grid_size_x * (ny + grid_size_y * nz);
                    int start = start_index[neighbor_cell_id];

                    if (start != -1)
                    {
                        for (size_t j = start; j < particles.size(); ++j)
                        {
                            if (particles[j].cell_id != neighbor_cell_id)
                                break;

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

// --- Core SPH Steps ---

void SPHSolver::compute_density_pressure()
{
#pragma omp parallel for
    for (int i = 0; i < particles.size(); ++i)
    {
        std::vector<int> neighbors;
        neighbors.reserve(100);

        particles[i].density = MASS * W_poly6(0.0f);

        find_neighbors(i, neighbors);
        for (int j : neighbors)
        {
            vec3 r = particles[i].pos - particles[j].pos;
            float r2 = dot(r, r);
            particles[i].density += MASS * W_poly6(r2);
        }

        if (particles[i].density < RHO0 * 0.01f)
        {
            particles[i].density = RHO0 * 0.01f;
        }

        particles[i].pressure = K * (glm::pow(particles[i].density / RHO0, 7.0f) - 1.0f);
        if (particles[i].pressure < 0.0f)
            particles[i].pressure = 0.0f;
    }
}

void SPHSolver::compute_forces()
{
#pragma omp parallel for
    for (int i = 0; i < particles.size(); ++i)
    {
        std::vector<int> neighbors;
        neighbors.reserve(100);

        vec3 F_pressure(0.0f);
        vec3 F_viscosity(0.0f);

        find_neighbors(i, neighbors);
        for (int j : neighbors)
        {
            if (i == j)
                continue;

            vec3 r = particles[i].pos - particles[j].pos;
            float r_len = length(r);

            float pressure_term = (particles[i].pressure + particles[j].pressure) / (2.0f * particles[j].density);
            F_pressure -= MASS * pressure_term * W_spiky_grad(r, r_len);

            vec3 vel_diff = particles[j].vel - particles[i].vel;
            F_viscosity += MASS * (vel_diff / particles[j].density) * MU * W_visc_lapl(r_len);
        }

        vec3 F_gravity = MASS * vec3(0.0f, -G, 0.0f);
        particles[i].acc = (F_pressure + F_viscosity + F_gravity) / MASS;
    }
}

void SPHSolver::integrate()
{
    const float MAX_VEL = 10.0f;

    for (auto &p : particles)
    {
        p.vel += p.acc * DT;

        float vel_len = glm::length(p.vel);
        if (vel_len > MAX_VEL)
        {
            p.vel = (p.vel / vel_len) * MAX_VEL;
        }

        if (std::isnan(p.vel.x) || std::isnan(p.vel.y) || std::isnan(p.vel.z))
        {
            p.vel = glm::vec3(0.0f);
        }
        if (std::isnan(p.pos.x) || std::isnan(p.pos.y) || std::isnan(p.pos.z))
        {
            p.pos = glm::vec3(1.0f);
        }

        p.pos += p.vel * DT;
    }
}

void SPHSolver::handle_boundary()
{
    float tank_max = 2.0f;
    float boundary_min = 0.01f;

    for (auto &p : particles)
    {
        // X-Axis
        if (p.pos.x < boundary_min)
        {
            p.pos.x = boundary_min;
            p.vel.x *= WALL_DAMPING;
        }
        else if (p.pos.x > tank_max - boundary_min)
        {
            p.pos.x = tank_max - boundary_min;
            p.vel.x *= WALL_DAMPING;
        }

        // Y-Axis
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

        // Z-Axis
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

// --- Main Update Loop ---

void SPHSolver::update()
{
    static int frame_count = 0;
    auto start = std::chrono::high_resolution_clock::now();

    update_spatial_hash();
    compute_density_pressure();
    compute_forces();
    integrate();
    handle_boundary();

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

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