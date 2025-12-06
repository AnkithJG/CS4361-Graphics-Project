#include "RayTracer.h"
#include <iostream>
#include <cmath>
#include <limits>
#include <omp.h>
#include <algorithm>

// constructor - allocate buffers and set up ray tracing
RayTracer::RayTracer(int width, int height)
    : width(width), height(height), device(nullptr), scene(nullptr), particle_radius(0.025f)
{
    aspect_ratio = (float)width / (float)height;

    // allocate depth buffers - 3x because we need 2 for separable filtering + 1 temp
    depth_buffer = new float[width * height * 3];
    temp_buffer = depth_buffer + (width * height * 2);

    initEmbree();

    // set up default camera
    setCamera(
        glm::vec3(3.0f, 2.5f, 3.0f),
        glm::vec3(1.0f, 1.0f, 1.0f),
        glm::vec3(0.0f, 1.0f, 0.0f),
        45.0f);

    std::cout << "RayTracer initialized: " << width << "x" << height << std::endl;
    std::cout << "Using screen-space fluid rendering" << std::endl;
}

// destructor - clean up
RayTracer::~RayTracer()
{
    delete[] depth_buffer;
    if (scene)
        rtcReleaseScene(scene);
    if (device)
        rtcReleaseDevice(device);
}

// initialize the embree ray tracing device
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

// set camera position and orientation
void RayTracer::setCamera(const glm::vec3 &pos, const glm::vec3 &target, const glm::vec3 &up, float fov_degrees)
{
    camera_pos = pos;
    this->fov = glm::radians(fov_degrees);

    // compute orthonormal camera basis using cross products
    camera_dir = glm::normalize(target - pos);
    camera_right = glm::normalize(glm::cross(camera_dir, up));
    camera_up = glm::normalize(glm::cross(camera_right, camera_dir));
}

// embree callback - test if ray hits a sphere
// this is called during ray tracing to check intersection
void RayTracer::sphereIntersectFunc(const struct RTCIntersectFunctionNArguments *args)
{
    int *valid = args->valid;
    void *ptr = args->geometryUserPtr;  // pointer to sphere array
    unsigned int primID = args->primID; // which sphere

    RTCRayHitN *rayhit = (RTCRayHitN *)args->rayhit;
    RTCRayN *rays = RTCRayHitN_RayN(rayhit, 1);
    RTCHitN *hits = RTCRayHitN_HitN(rayhit, 1);

    Sphere *sphere = &((Sphere *)ptr)[primID];

    // extract ray origin
    float ox = RTCRayN_org_x(rays, 1, 0);
    float oy = RTCRayN_org_y(rays, 1, 0);
    float oz = RTCRayN_org_z(rays, 1, 0);

    // extract ray direction
    float dx = RTCRayN_dir_x(rays, 1, 0);
    float dy = RTCRayN_dir_y(rays, 1, 0);
    float dz = RTCRayN_dir_z(rays, 1, 0);

    // t range (tnear is closest, tfar is farthest allowed)
    float tnear = RTCRayN_tnear(rays, 1, 0);
    float tfar = RTCRayN_tfar(rays, 1, 0);

    // vector from ray origin to sphere center
    glm::vec3 oc(ox - sphere->center.x, oy - sphere->center.y, oz - sphere->center.z);
    glm::vec3 dir(dx, dy, dz);

    // standard sphere-ray intersection formula
    // ray = origin + t*direction, sphere = ||p - center|| = radius
    // substitute and solve quadratic: a*t² + b*t + c = 0
    float a = glm::dot(dir, dir);
    float b = 2.0f * glm::dot(oc, dir);
    float c = glm::dot(oc, oc) - sphere->radius * sphere->radius;
    float discriminant = b * b - 4.0f * a * c;

    if (discriminant >= 0.0f)
    {
        // take closer intersection point
        float t = (-b - sqrt(discriminant)) / (2.0f * a);

        if (t >= tnear && t <= tfar)
        {
            // record hit
            RTCRayN_tfar(rays, 1, 0) = t;
            RTCHitN_geomID(hits, 1, 0) = args->geomID;
            RTCHitN_primID(hits, 1, 0) = primID;

            // compute normal at hit point (for a sphere, normal = (point - center).normalize())
            glm::vec3 hitpoint = oc + t * dir;
            glm::vec3 normal = glm::normalize(hitpoint);
            RTCHitN_Ng_x(hits, 1, 0) = normal.x;
            RTCHitN_Ng_y(hits, 1, 0) = normal.y;
            RTCHitN_Ng_z(hits, 1, 0) = normal.z;
        }
    }
}

// return bounding box for sphere (used for acceleration)
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

// test if ray is blocked by sphere (for shadow rays)
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
            // mark ray as occluded (negative infinity t means blocked)
            RTCRayN_tfar(rays, 1, 0) = -std::numeric_limits<float>::infinity();
        }
    }
}

// update particle spheres from positions
void RayTracer::updateScene(const std::vector<glm::vec3> &particle_positions, float particle_radius)
{
    this->particle_radius = particle_radius;

    // delete old scene
    if (scene)
    {
        rtcReleaseScene(scene);
    }

    // create new scene
    scene = rtcNewScene(device);
    spheres.clear();
    spheres.reserve(particle_positions.size());

    // convert particle positions to spheres
    for (const auto &pos : particle_positions)
    {
        Sphere s;
        s.center = pos;
        s.radius = particle_radius * 12.0f;    // scale up for visibility
        s.color = glm::vec3(0.3f, 0.6f, 0.9f); // blue-ish color
        spheres.push_back(s);
    }

    buildScene();
}

// create embree geometry from spheres
void RayTracer::buildScene()
{
    if (spheres.empty())
        return;

    // create user-defined geometry (custom intersection)
    RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_USER);

    // set up callbacks
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

// generate ray direction from pixel coordinates
glm::vec3 RayTracer::generateRayDirection(int pixel_x, int pixel_y)
{
    // convert pixel coordinates to normalized device coordinates (-1 to 1)
    float ndc_x = (2.0f * pixel_x / width - 1.0f) * aspect_ratio;
    float ndc_y = 1.0f - 2.0f * pixel_y / height;

    float tan_fov = tan(fov / 2.0f);
    // construct ray direction in camera space
    glm::vec3 ray_dir = camera_dir +
                        ndc_x * tan_fov * camera_right +
                        ndc_y * tan_fov * camera_up;

    return glm::normalize(ray_dir);
}

// trace rays from camera and get depth (distance) to particles
void RayTracer::renderDepthPass(float *depth_buffer)
{
    if (!scene)
        return;

    const float far_depth = 1000.0f;

    // initialize all pixels to far depth
#pragma omp parallel for
    for (int i = 0; i < width * height; ++i)
    {
        depth_buffer[i] = far_depth;
    }

    // trace a ray for each pixel
#pragma omp parallel for schedule(dynamic)
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            glm::vec3 ray_dir = generateRayDirection(x, y);

            // set up ray
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

            // trace it
            rtcIntersect1(scene, &rayhit);

            // record depth if we hit something
            if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID)
            {
                depth_buffer[y * width + x] = rayhit.ray.tfar;
            }
        }
    }
}

void RayTracer::smoothDepthBuffer(float *depth_in, float *depth_out)
{
    const int kernel_size = 11;
    const int half_kernel = kernel_size / 2;
    const float far_depth = 1000.0f;
    const float depth_threshold = 0.3f;

#pragma omp parallel for schedule(dynamic)
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            float center_depth = depth_in[y * width + x];

            if (center_depth >= far_depth * 0.9f)
            {
                depth_out[y * width + x] = far_depth;
                continue;
            }

            float sum_depth = 0.0f;
            float sum_weight = 0.0f;

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

void RayTracer::smoothDepthBufferSeparable(float *depth_in, float *depth_out)
{
    const int kernel_radius = 12;
    const float far_depth = 1000.0f;
    const float depth_threshold = 0.4f;

    int w = width;
    int h = height;

#pragma omp parallel for schedule(dynamic)
    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            float center_depth = depth_in[y * w + x];

            if (center_depth >= far_depth * 0.9f)
            {
                temp_buffer[y * w + x] = far_depth;
                continue;
            }

            float sum_depth = 0.0f;
            float sum_weight = 0.0f;

            for (int kx = -kernel_radius; kx <= kernel_radius; ++kx)
            {
                int nx = x + kx;
                if (nx >= 0 && nx < w)
                {
                    float neighbor_depth = depth_in[y * w + nx];

                    if (neighbor_depth < far_depth * 0.9f)
                    {
                        float depth_diff = abs(neighbor_depth - center_depth);
                        if (depth_diff < depth_threshold)
                        {
                            sum_depth += neighbor_depth;
                            sum_weight += 1.0f;
                        }
                    }
                }
            }

            temp_buffer[y * w + x] = (sum_weight > 0.0f) ? (sum_depth / sum_weight) : center_depth;
        }
    }

#pragma omp parallel for schedule(dynamic)
    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            float center_depth = temp_buffer[y * w + x];

            if (center_depth >= far_depth * 0.9f)
            {
                depth_out[y * w + x] = far_depth;
                continue;
            }

            float sum_depth = 0.0f;
            float sum_weight = 0.0f;

            for (int ky = -kernel_radius; ky <= kernel_radius; ++ky)
            {
                int ny = y + ky;
                if (ny >= 0 && ny < h)
                {
                    float neighbor_depth = temp_buffer[ny * w + x];

                    if (neighbor_depth < far_depth * 0.9f)
                    {
                        float depth_diff = abs(neighbor_depth - center_depth);
                        if (depth_diff < depth_threshold)
                        {
                            sum_depth += neighbor_depth;
                            sum_weight += 1.0f;
                        }
                    }
                }
            }

            depth_out[y * w + x] = (sum_weight > 0.0f) ? (sum_depth / sum_weight) : center_depth;
        }
    }
}

glm::vec3 RayTracer::reconstructWorldPosition(int x, int y, float depth)
{
    glm::vec3 ray_dir = generateRayDirection(x, y);
    return camera_pos + ray_dir * depth;
}

glm::vec3 RayTracer::computeNormalFromDepth(int x, int y, float *depth_buffer)
{
    const float far_depth = 1000.0f;
    float center_depth = depth_buffer[y * width + x];

    if (center_depth >= far_depth * 0.9f)
        return glm::vec3(0, 1, 0);

    float depth_right = (x < width - 1) ? depth_buffer[y * width + (x + 1)] : center_depth;
    float depth_left = (x > 0) ? depth_buffer[y * width + (x - 1)] : center_depth;
    float depth_up = (y > 0) ? depth_buffer[(y - 1) * width + x] : center_depth;
    float depth_down = (y < height - 1) ? depth_buffer[(y + 1) * width + x] : center_depth;

    glm::vec3 pos_center = reconstructWorldPosition(x, y, center_depth);
    glm::vec3 pos_right = reconstructWorldPosition(x + 1, y, depth_right);
    glm::vec3 pos_up = reconstructWorldPosition(x, y - 1, depth_up);

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
                glm::vec3 surface_point = reconstructWorldPosition(x, y, depth);
                glm::vec3 normal = computeNormalFromDepth(x, y, depth_buffer);

                color = shadeFluid(surface_point, normal);
            }
            else
            {
                color = glm::vec3(0.0f, 0.0f, 0.0f);
            }

            int index = (y * width + x) * 3;
            framebuffer[index + 0] = (unsigned char)(glm::clamp(color.r, 0.0f, 1.0f) * 255);
            framebuffer[index + 1] = (unsigned char)(glm::clamp(color.g, 0.0f, 1.0f) * 255);
            framebuffer[index + 2] = (unsigned char)(glm::clamp(color.b, 0.0f, 1.0f) * 255);
        }
    }
}

void RayTracer::render(unsigned char *framebuffer)
{
    if (!scene)
    {
        std::cerr << "ERROR: No scene to render!" << std::endl;
        return;
    }

    float *depth_buffer_1 = depth_buffer;
    float *depth_buffer_2 = depth_buffer + (width * height);

    renderDepthPass(depth_buffer_1);

    smoothDepthBufferSeparable(depth_buffer_1, depth_buffer_2);
    smoothDepthBufferSeparable(depth_buffer_2, depth_buffer_1);
    smoothDepthBufferSeparable(depth_buffer_1, depth_buffer_2);
    smoothDepthBufferSeparable(depth_buffer_2, depth_buffer_1);
    smoothDepthBufferSeparable(depth_buffer_1, depth_buffer_2);

    renderFluidSurface(framebuffer, depth_buffer_2);
}