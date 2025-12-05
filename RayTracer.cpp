// RayTracer.cpp

#include "RayTracer.h"
#include <iostream>
#include <cmath>
#include <limits>
#include <omp.h>

// Constructor
RayTracer::RayTracer(int width, int height)
    : width(width), height(height), device(nullptr), scene(nullptr)
{
    aspect_ratio = (float)width / (float)height;
    initEmbree();

    // Default camera setup
    setCamera(
        glm::vec3(3.0f, 2.5f, 3.0f), // position
        glm::vec3(1.0f, 1.0f, 1.0f), // look at
        glm::vec3(0.0f, 1.0f, 0.0f), // up
        45.0f                        // fov
    );

    std::cout << "RayTracer initialized: " << width << "x" << height << std::endl;
}

// Destructor
RayTracer::~RayTracer()
{
    if (scene)
        rtcReleaseScene(scene);
    if (device)
        rtcReleaseDevice(device);
}

// Initialize Embree
void RayTracer::initEmbree()
{
    device = rtcNewDevice(nullptr);
    if (!device)
    {
        std::cerr << "ERROR: Failed to create Embree device!" << std::endl;
        return;
    }

    // Check for errors
    RTCError error = rtcGetDeviceError(device);
    if (error != RTC_ERROR_NONE)
    {
        std::cerr << "Embree device error: " << error << std::endl;
    }

    std::cout << "Embree device created successfully" << std::endl;
}

// Set up camera
void RayTracer::setCamera(const glm::vec3 &pos, const glm::vec3 &target, const glm::vec3 &up, float fov_degrees)
{
    camera_pos = pos;
    this->fov = glm::radians(fov_degrees);

    // Calculate camera basis vectors
    camera_dir = glm::normalize(target - pos);
    camera_right = glm::normalize(glm::cross(camera_dir, up));
    camera_up = glm::normalize(glm::cross(camera_right, camera_dir));

    std::cout << "Camera set at (" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
}

// Sphere intersection callback for Embree
void RayTracer::sphereIntersectFunc(const struct RTCIntersectFunctionNArguments *args)
{
    int *valid = args->valid;
    void *ptr = args->geometryUserPtr;
    unsigned int primID = args->primID;

    RTCRayHitN *rayhit = (RTCRayHitN *)args->rayhit;
    RTCRayN *rays = RTCRayHitN_RayN(rayhit, 1);
    RTCHitN *hits = RTCRayHitN_HitN(rayhit, 1);

    Sphere *sphere = &((Sphere *)ptr)[primID];

    // Get ray data
    float ox = RTCRayN_org_x(rays, 1, 0);
    float oy = RTCRayN_org_y(rays, 1, 0);
    float oz = RTCRayN_org_z(rays, 1, 0);
    float dx = RTCRayN_dir_x(rays, 1, 0);
    float dy = RTCRayN_dir_y(rays, 1, 0);
    float dz = RTCRayN_dir_z(rays, 1, 0);
    float tnear = RTCRayN_tnear(rays, 1, 0);
    float tfar = RTCRayN_tfar(rays, 1, 0);

    // Ray-sphere intersection
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
            // Update hit information
            RTCRayN_tfar(rays, 1, 0) = t;
            RTCHitN_u(hits, 1, 0) = 0.0f;
            RTCHitN_v(hits, 1, 0) = 0.0f;
            RTCHitN_geomID(hits, 1, 0) = args->geomID;
            RTCHitN_primID(hits, 1, 0) = primID;

            // Calculate normal
            glm::vec3 hitpoint = oc + t * dir;
            glm::vec3 normal = glm::normalize(hitpoint);
            RTCHitN_Ng_x(hits, 1, 0) = normal.x;
            RTCHitN_Ng_y(hits, 1, 0) = normal.y;
            RTCHitN_Ng_z(hits, 1, 0) = normal.z;
        }
    }
}

// Sphere bounds callback for Embree
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

// Sphere occlusion callback (for shadow rays)
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

// Update scene with particle positions
void RayTracer::updateScene(const std::vector<glm::vec3> &particle_positions, float particle_radius)
{
    // Release old scene
    if (scene)
    {
        rtcReleaseScene(scene);
    }

    // Create new scene
    scene = rtcNewScene(device);

    // Convert particles to spheres
    spheres.clear();
    spheres.reserve(particle_positions.size());

    for (const auto &pos : particle_positions)
    {
        Sphere s;
        s.center = pos;
        s.radius = particle_radius;
        s.color = glm::vec3(0.3f, 0.6f, 0.9f); // Blue water color
        spheres.push_back(s);
    }

    buildScene();

    std::cout << "Scene updated with " << spheres.size() << " spheres" << std::endl;
}

// Build Embree scene from spheres
void RayTracer::buildScene()
{
    if (spheres.empty())
        return;

    // Create user geometry for spheres
    RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_USER);

    // Set bounds and intersection functions
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

// Generate ray direction for a pixel
glm::vec3 RayTracer::generateRayDirection(int pixel_x, int pixel_y)
{
    // Normalized device coordinates [-1, 1]
    float ndc_x = (2.0f * pixel_x / width - 1.0f) * aspect_ratio;
    float ndc_y = 1.0f - 2.0f * pixel_y / height;

    // Calculate ray direction in camera space
    float tan_fov = tan(fov / 2.0f);
    glm::vec3 ray_dir = camera_dir +
                        ndc_x * tan_fov * camera_right +
                        ndc_y * tan_fov * camera_up;

    return glm::normalize(ray_dir);
}

// Simple shading function
glm::vec3 RayTracer::shade(const RTCRayHit &rayhit)
{
    // Check if we hit something
    if (rayhit.hit.geomID == RTC_INVALID_GEOMETRY_ID)
    {
        // Background color (black)
        return glm::vec3(0.0f, 0.0f, 0.0f);
    }

    // Get sphere that was hit
    const Sphere &sphere = spheres[rayhit.hit.primID];

    // Get surface normal
    glm::vec3 normal(rayhit.hit.Ng_x, rayhit.hit.Ng_y, rayhit.hit.Ng_z);
    normal = glm::normalize(normal);

    // Simple directional light from above
    glm::vec3 light_dir = glm::normalize(glm::vec3(0.3f, 1.0f, 0.3f));

    // Diffuse shading
    float diffuse = std::max(0.0f, glm::dot(normal, light_dir));

    // Ambient + diffuse
    glm::vec3 ambient = sphere.color * 0.3f;
    glm::vec3 color = ambient + sphere.color * diffuse * 0.7f;

    return color;
}

// Main render function
void RayTracer::render(unsigned char *framebuffer)
{
    if (!scene)
    {
        std::cerr << "ERROR: No scene to render!" << std::endl;
        return;
    }

// Parallel ray tracing
#pragma omp parallel for schedule(dynamic)
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            // Generate ray
            glm::vec3 ray_dir = generateRayDirection(x, y);

            // Setup Embree ray
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

            // Trace ray
            rtcIntersect1(scene, &rayhit);

            // Shade pixel
            glm::vec3 color = shade(rayhit);

            // Write to framebuffer (RGB)
            int index = (y * width + x) * 3;
            framebuffer[index + 0] = (unsigned char)(glm::clamp(color.r, 0.0f, 1.0f) * 255);
            framebuffer[index + 1] = (unsigned char)(glm::clamp(color.g, 0.0f, 1.0f) * 255);
            framebuffer[index + 2] = (unsigned char)(glm::clamp(color.b, 0.0f, 1.0f) * 255);
        }
    }
}