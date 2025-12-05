// RayTracer.h

#ifndef RAYTRACER_H
#define RAYTRACER_H

#include <embree4/rtcore.h>
#include <glm/glm.hpp>
#include <vector>

struct Sphere
{
    glm::vec3 center;
    float radius;
    glm::vec3 color;
};

class RayTracer
{
public:
    RayTracer(int width, int height);
    ~RayTracer();

    // Camera setup
    void setCamera(const glm::vec3 &pos, const glm::vec3 &target, const glm::vec3 &up, float fov);

    // Scene building - convert SPH particles to spheres
    void updateScene(const std::vector<glm::vec3> &particle_positions, float particle_radius);

    // Main rendering function - outputs to RGB buffer
    void render(unsigned char *framebuffer);

    // Get current resolution
    int getWidth() const { return width; }
    int getHeight() const { return height; }

private:
    // Embree context
    RTCDevice device;
    RTCScene scene;

    // Camera parameters
    glm::vec3 camera_pos;
    glm::vec3 camera_dir;
    glm::vec3 camera_up;
    glm::vec3 camera_right;
    float fov;
    float aspect_ratio;

    // Screen resolution
    int width, height;

    // Scene geometry storage
    std::vector<Sphere> spheres;

    // Helper functions
    void initEmbree();
    void buildScene();
    glm::vec3 generateRayDirection(int pixel_x, int pixel_y);
    glm::vec3 shade(const RTCRayHit &rayhit);

    // Embree callbacks for sphere intersection
    static void sphereBoundsFunc(const struct RTCBoundsFunctionArguments *args);
    static void sphereIntersectFunc(const struct RTCIntersectFunctionNArguments *args);
    static void sphereOccludedFunc(const struct RTCOccludedFunctionNArguments *args);
};

#endif // RAYTRACER_H