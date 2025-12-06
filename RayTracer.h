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

    void setCamera(const glm::vec3 &pos, const glm::vec3 &target, const glm::vec3 &up, float fov);

    void updateScene(const std::vector<glm::vec3> &particle_positions, float particle_radius);

    void render(unsigned char *framebuffer);

    int getWidth() const { return width; }
    int getHeight() const { return height; }

private:
    RTCDevice device;
    RTCScene scene;

    glm::vec3 camera_pos;
    glm::vec3 camera_dir;
    glm::vec3 camera_up;
    glm::vec3 camera_right;
    float fov;
    float aspect_ratio;

    int width, height;

    std::vector<Sphere> spheres;
    float particle_radius;

    float *depth_buffer;
    float *temp_buffer;

    void initEmbree();
    void buildScene();

    void renderDepthPass(float *depth_buffer);
    void smoothDepthBuffer(float *depth_in, float *depth_out);
    void smoothDepthBufferSeparable(float *depth_in, float *depth_out);
    void renderFluidSurface(unsigned char *framebuffer, float *depth_buffer);

    glm::vec3 generateRayDirection(int pixel_x, int pixel_y);
    glm::vec3 reconstructWorldPosition(int x, int y, float depth);
    glm::vec3 computeNormalFromDepth(int x, int y, float *depth_buffer);
    glm::vec3 shadeFluid(const glm::vec3 &surface_point, const glm::vec3 &normal);

    static void sphereBoundsFunc(const struct RTCBoundsFunctionArguments *args);
    static void sphereIntersectFunc(const struct RTCIntersectFunctionNArguments *args);
    static void sphereOccludedFunc(const struct RTCOccludedFunctionNArguments *args);
};

#endif
