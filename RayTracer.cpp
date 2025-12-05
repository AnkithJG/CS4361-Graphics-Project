// RayTracer.cpp - Screen-Space Fluid Rendering

#include "RayTracer.h"
#include <iostream>
#include <cmath>
#include <limits>
#include <omp.h>
#include <algorithm>

RayTracer::RayTracer(int width, int height)
    : width(width), height(height), device(nullptr), scene(nullptr), particle_radius(0.025f)
{
    aspect_ratio = (float)width / (float)height;

    // Allocate depth buffer
    depth_buffer = new float[width * height * 2]; // Double buffer for smoothing

    initEmbree();

    setCamera(
        glm::vec3(3.0f, 2.5f, 3.0f),
        glm::vec3(1.0f, 1.0f, 1.0f),
        glm::vec3(0.0f, 1.0f, 0.0f),
        45.0f);

    std::cout << "RayTracer initialized: " << width << "x" << height << std::endl;
    std::cout << "Using screen-space fluid rendering" << std::endl;
}

RayTracer::~RayTracer()
{
    delete[] depth_buffer;
    if (scene)
        rtcReleaseScene(scene);
    if (device)
        rtcReleaseDevice(device);
}

void RayTracer::initEmbree()
{
    device = rtcNewDevice(nullptr);
    if (!device)
    {
        std::cerr << "ERROR: Failed to create Embree device!" << std::endl;
        return;
    }

    RTCError error = rtcGetDeviceError(device);
    if (error != RTC_ERROR_NONE)
    {
        std::cerr << "Embree device error: " << error << std::endl;
    }

    std::cout << "Embree device created successfully" << std::endl;
}

void RayTracer::setCamera(const glm::vec3 &pos, const glm::vec3 &target, const glm::vec3 &up, float fov_degrees)
{
    camera_pos = pos;
    this->fov = glm::radians(fov_degrees);

    camera_dir = glm::normalize(target - pos);
    camera_right = glm::normalize(glm::cross(camera_dir, up));
    camera_up = glm::normalize(glm::cross(camera_right, camera_dir));
}

void RayTracer::sphereIntersectFunc(const struct RTCIntersectFunctionNArguments *args)
{
    int *valid = args->valid;
    void *ptr = args->geometryUserPtr;
    unsigned int primID = args->primID;

    RTCRayHitN *rayhit = (RTCRayHitN *)args->rayhit;
    RTCRayN *rays = RTCRayHitN_RayN(rayhit, 1);
    RTCHitN *hits = RTCRayHitN_HitN(rayhit, 1);

    Sphere *sphere = &((Sphere *)ptr)[primID];

    float ox = RTCRayN_org_x(rays, 1, 0);
    float oy = RTCRayN_org_y(rays, 1, 0);
    float oz = RTCRayN_org_z(rays, 1, 0);
    float dx = RTCRayN_dir_x(rays, 1, 0);
    float dy = RTCRayN_dir_y(rays, 1, 0);
    float dz = RTCRayN_dir_z(rays, 1, 0);
    float tnear = RTCRayN_tnear(rays, 1, 0);
    float tfar = RTCRayN_tfar(rays, 1, 0);

    glm::vec3 oc(ox - sphere->center.x, oy - sphere->center.y, oz - sphere->center.z);
    glm::vec3 dir(dx, dy, dz);

    float a = glm::dot(dir, dir);
    float b = 2.0f * glm::dot(oc, dir);
    float c = glm::dot(oc, oc) - sphere->radius * sphere->radius;
    float discriminant = b * b - 4.0f * a * c;

    if (discriminant >= 0.0f)
    {
        float t = (-b - sqrt(discriminant)) / (2.0f * a);

        if (t >= tnear && t <= tfar)
        {
            RTCRayN_tfar(rays, 1, 0) = t;
            RTCHitN_geomID(hits, 1, 0) = args->geomID;
            RTCHitN_primID(hits, 1, 0) = primID;

            glm::vec3 hitpoint = oc + t * dir;
            glm::vec3 normal = glm::normalize(hitpoint);
            RTCHitN_Ng_x(hits, 1, 0) = normal.x;
            RTCHitN_Ng_y(hits, 1, 0) = normal.y;
            RTCHitN_Ng_z(hits, 1, 0) = normal.z;
        }
    }
}

void RayTracer::sphereBoundsFunc(const struct RTCBoundsFunctionArguments *args)
{
    const Sphere *spheres = (const Sphere *)args->geometryUserPtr;
    const Sphere &sphere = spheres[args->primID];

    RTCBounds *bounds_o = args->bounds_o;
    bounds_o->lower_x = sphere.center.x - sphere.radius;
    bounds_o->lower_y = sphere.center.y - sphere.radius;
    bounds_o->lower_z = sphere.center.z - sphere.radius;
    bounds_o->upper_x = sphere.center.x + sphere.radius;
    bounds_o->upper_y = sphere.center.y + sphere.radius;
    bounds_o->upper_z = sphere.center.z + sphere.radius;
}

void RayTracer::sphereOccludedFunc(const struct RTCOccludedFunctionNArguments *args)
{
    int *valid = args->valid;
    void *ptr = args->geometryUserPtr;
    unsigned int primID = args->primID;

    RTCRayN *rays = (RTCRayN *)args->ray;
    Sphere *sphere = &((Sphere *)ptr)[primID];

    float ox = RTCRayN_org_x(rays, 1, 0);
    float oy = RTCRayN_org_y(rays, 1, 0);
    float oz = RTCRayN_org_z(rays, 1, 0);
    float dx = RTCRayN_dir_x(rays, 1, 0);
    float dy = RTCRayN_dir_y(rays, 1, 0);
    float dz = RTCRayN_dir_z(rays, 1, 0);
    float tnear = RTCRayN_tnear(rays, 1, 0);
    float tfar = RTCRayN_tfar(rays, 1, 0);

    glm::vec3 oc(ox - sphere->center.x, oy - sphere->center.y, oz - sphere->center.z);
    glm::vec3 dir(dx, dy, dz);

    float a = glm::dot(dir, dir);
    float b = 2.0f * glm::dot(oc, dir);
    float c = glm::dot(oc, oc) - sphere->radius * sphere->radius;
    float discriminant = b * b - 4.0f * a * c;

    if (discriminant >= 0.0f)
    {
        float t = (-b - sqrt(discriminant)) / (2.0f * a);
        if (t >= tnear && t <= tfar)
        {
            RTCRayN_tfar(rays, 1, 0) = -std::numeric_limits<float>::infinity();
        }
    }
}

void RayTracer::updateScene(const std::vector<glm::vec3> &particle_positions, float particle_radius)
{
    this->particle_radius = particle_radius;

    if (scene)
    {
        rtcReleaseScene(scene);
    }

    scene = rtcNewScene(device);
    spheres.clear();
    spheres.reserve(particle_positions.size());

    for (const auto &pos : particle_positions)
    {
        Sphere s;
        s.center = pos;
        s.radius = particle_radius * 4.5f; // MUCH bigger overlap for fluid look
        s.color = glm::vec3(0.3f, 0.6f, 0.9f);
        spheres.push_back(s);
    }

    buildScene();
}

void RayTracer::buildScene()
{
    if (spheres.empty())
        return;

    RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_USER);

    rtcSetGeometryUserPrimitiveCount(geom, spheres.size());
    rtcSetGeometryUserData(geom, spheres.data());
    rtcSetGeometryBoundsFunction(geom, sphereBoundsFunc, nullptr);
    rtcSetGeometryIntersectFunction(geom, sphereIntersectFunc);
    rtcSetGeometryOccludedFunction(geom, sphereOccludedFunc);

    rtcCommitGeometry(geom);
    rtcAttachGeometry(scene, geom);
    rtcReleaseGeometry(geom);

    rtcCommitScene(scene);
}

glm::vec3 RayTracer::generateRayDirection(int pixel_x, int pixel_y)
{
    float ndc_x = (2.0f * pixel_x / width - 1.0f) * aspect_ratio;
    float ndc_y = 1.0f - 2.0f * pixel_y / height;

    float tan_fov = tan(fov / 2.0f);
    glm::vec3 ray_dir = camera_dir +
                        ndc_x * tan_fov * camera_right +
                        ndc_y * tan_fov * camera_up;

    return glm::normalize(ray_dir);
}

// Pass 1: Render depth values of sphere intersections
void RayTracer::renderDepthPass(float *depth_buffer)
{
    if (!scene)
        return;

    const float far_depth = 1000.0f;

// Initialize depth buffer
#pragma omp parallel for
    for (int i = 0; i < width * height; ++i)
    {
        depth_buffer[i] = far_depth;
    }

// Ray trace to get depths
#pragma omp parallel for schedule(dynamic)
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            glm::vec3 ray_dir = generateRayDirection(x, y);

            RTCRayHit rayhit;
            rayhit.ray.org_x = camera_pos.x;
            rayhit.ray.org_y = camera_pos.y;
            rayhit.ray.org_z = camera_pos.z;
            rayhit.ray.dir_x = ray_dir.x;
            rayhit.ray.dir_y = ray_dir.y;
            rayhit.ray.dir_z = ray_dir.z;
            rayhit.ray.tnear = 0.0f;
            rayhit.ray.tfar = std::numeric_limits<float>::infinity();
            rayhit.ray.mask = 0xFFFFFFFF;
            rayhit.ray.flags = 0;
            rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
            rayhit.hit.primID = RTC_INVALID_GEOMETRY_ID;

            rtcIntersect1(scene, &rayhit);

            if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID)
            {
                depth_buffer[y * width + x] = rayhit.ray.tfar;
            }
        }
    }
}

// Pass 2: Smooth depth buffer (bilateral filter)
void RayTracer::smoothDepthBuffer(float *depth_in, float *depth_out)
{
    const int kernel_size = 11; // Bigger kernel for more blending
    const int half_kernel = kernel_size / 2;
    const float far_depth = 1000.0f;
    const float depth_threshold = 0.3f; // Very tolerant - blend aggressively

#pragma omp parallel for schedule(dynamic)
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            float center_depth = depth_in[y * width + x];

            // Don't smooth background
            if (center_depth >= far_depth * 0.9f)
            {
                depth_out[y * width + x] = far_depth;
                continue;
            }

            float sum_depth = 0.0f;
            float sum_weight = 0.0f;

            // Bilateral filter: smooth while preserving edges
            for (int ky = -half_kernel; ky <= half_kernel; ++ky)
            {
                for (int kx = -half_kernel; kx <= half_kernel; ++kx)
                {
                    int nx = x + kx;
                    int ny = y + ky;

                    if (nx >= 0 && nx < width && ny >= 0 && ny < height)
                    {
                        float neighbor_depth = depth_in[ny * width + nx];

                        if (neighbor_depth < far_depth * 0.9f)
                        {
                            // Simplified weight calculation for speed
                            float depth_diff = abs(neighbor_depth - center_depth);

                            if (depth_diff < depth_threshold)
                            {
                                sum_depth += neighbor_depth;
                                sum_weight += 1.0f;
                            }
                        }
                    }
                }
            }

            depth_out[y * width + x] = (sum_weight > 0.0f) ? (sum_depth / sum_weight) : center_depth;
        }
    }
}

// Reconstruct 3D position from screen coordinates and depth
glm::vec3 RayTracer::reconstructWorldPosition(int x, int y, float depth)
{
    glm::vec3 ray_dir = generateRayDirection(x, y);
    return camera_pos + ray_dir * depth;
}

// Compute normal from depth buffer using finite differences
glm::vec3 RayTracer::computeNormalFromDepth(int x, int y, float *depth_buffer)
{
    const float far_depth = 1000.0f;
    float center_depth = depth_buffer[y * width + x];

    if (center_depth >= far_depth * 0.9f)
        return glm::vec3(0, 1, 0);

    // Get neighboring depths
    float depth_right = (x < width - 1) ? depth_buffer[y * width + (x + 1)] : center_depth;
    float depth_left = (x > 0) ? depth_buffer[y * width + (x - 1)] : center_depth;
    float depth_up = (y > 0) ? depth_buffer[(y - 1) * width + x] : center_depth;
    float depth_down = (y < height - 1) ? depth_buffer[(y + 1) * width + x] : center_depth;

    // Reconstruct 3D positions
    glm::vec3 pos_center = reconstructWorldPosition(x, y, center_depth);
    glm::vec3 pos_right = reconstructWorldPosition(x + 1, y, depth_right);
    glm::vec3 pos_up = reconstructWorldPosition(x, y - 1, depth_up);

    // Compute normal from cross product
    glm::vec3 dx = pos_right - pos_center;
    glm::vec3 dy = pos_up - pos_center;
    glm::vec3 normal = glm::normalize(glm::cross(dx, dy));

    return normal;
}

glm::vec3 RayTracer::shadeFluid(const glm::vec3 &surface_point, const glm::vec3 &normal)
{
    glm::vec3 base_color(0.1f, 0.35f, 0.7f);

    glm::vec3 light1_dir = glm::normalize(glm::vec3(0.5f, 1.0f, 0.3f));
    glm::vec3 light2_dir = glm::normalize(glm::vec3(-0.3f, 0.5f, -0.5f));

    float diff1 = std::max(0.0f, glm::dot(normal, light1_dir));
    float diff2 = std::max(0.0f, glm::dot(normal, light2_dir)) * 0.4f;

    glm::vec3 view_dir = glm::normalize(camera_pos - surface_point);
    glm::vec3 half_vec1 = glm::normalize(light1_dir + view_dir);
    float spec1 = pow(std::max(0.0f, glm::dot(normal, half_vec1)), 80.0f);

    float fresnel = pow(1.0f - std::max(0.0f, glm::dot(normal, view_dir)), 3.5f);

    glm::vec3 ambient = base_color * 0.3f;
    glm::vec3 diffuse = base_color * (diff1 * 0.6f + diff2 * 0.3f);
    glm::vec3 specular = glm::vec3(1.0f, 1.0f, 1.0f) * spec1 * 0.8f;
    glm::vec3 rim = glm::vec3(0.6f, 0.8f, 1.0f) * fresnel * 0.5f;

    return ambient + diffuse + specular + rim;
}

// Pass 3: Render final fluid surface from smoothed depth
void RayTracer::renderFluidSurface(unsigned char *framebuffer, float *depth_buffer)
{
    const float far_depth = 1000.0f;

#pragma omp parallel for schedule(dynamic)
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            float depth = depth_buffer[y * width + x];
            glm::vec3 color;

            if (depth < far_depth * 0.9f)
            {
                // Reconstruct surface point and compute normal
                glm::vec3 surface_point = reconstructWorldPosition(x, y, depth);
                glm::vec3 normal = computeNormalFromDepth(x, y, depth_buffer);

                color = shadeFluid(surface_point, normal);
            }
            else
            {
                // Background
                color = glm::vec3(0.0f, 0.0f, 0.0f);
            }

            int index = (y * width + x) * 3;
            framebuffer[index + 0] = (unsigned char)(glm::clamp(color.r, 0.0f, 1.0f) * 255);
            framebuffer[index + 1] = (unsigned char)(glm::clamp(color.g, 0.0f, 1.0f) * 255);
            framebuffer[index + 2] = (unsigned char)(glm::clamp(color.b, 0.0f, 1.0f) * 255);
        }
    }
}

// Main render function - three pass pipeline
void RayTracer::render(unsigned char *framebuffer)
{
    if (!scene)
    {
        std::cerr << "ERROR: No scene to render!" << std::endl;
        return;
    }

    float *depth_buffer_1 = depth_buffer;
    float *depth_buffer_2 = depth_buffer + (width * height);

    // Pass 1: Render sphere depths
    renderDepthPass(depth_buffer_1);

    // Pass 2: Smooth depth buffer TWICE for more fluid look
    smoothDepthBuffer(depth_buffer_1, depth_buffer_2);
    smoothDepthBuffer(depth_buffer_2, depth_buffer_1); // Second pass!

    // Pass 3: Render final fluid surface
    renderFluidSurface(framebuffer, depth_buffer_1);
}