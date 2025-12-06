#ifndef RAYTRACER_H
#define RAYTRACER_H

#include <embree4/rtcore.h>
#include <glm/glm.hpp>
#include <vector>

// represents a sphere (particle) for ray tracing
struct Sphere
{
    glm::vec3 center; // position
    float radius;     // size
    glm::vec3 color;  // rgb color
};

// handles rendering the fluid simulation using ray tracing
// basically traces rays from camera to find particles and renders them as spheres
class RayTracer
{
public:
    RayTracer(int width, int height);
    ~RayTracer();

    void setCamera(const glm::vec3 &pos, const glm::vec3 &target, const glm::vec3 &up, float fov);

    void updateScene(const std::vector<glm::vec3> &particle_positions, float particle_radius);

    void render(unsigned char *framebuffer); // render to image buffer

    int getWidth() const { return width; }
    int getHeight() const { return height; }

private:
    // embree ray tracing library stuff
    RTCDevice device; // gpu/cpu device for ray tracing
    RTCScene scene;   // the scene (contains all spheres)

    // camera parameters
    glm::vec3 camera_pos;   // where camera is
    glm::vec3 camera_dir;   // what it's looking at
    glm::vec3 camera_up;    // which way is up
    glm::vec3 camera_right; // perpendicular to up
    float fov;              // field of view
    float aspect_ratio;     // width/height ratio

    int width, height; // screen resolution

    std::vector<Sphere> spheres; // all particle spheres to render
    float particle_radius;

    // depth buffers for screen-space rendering
    // basically stores depth (distance) for each pixel
    float *depth_buffer; // allocated as 3x width*height floats
    float *temp_buffer;  // second buffer for separable filtering

    // ray tracing setup
    void initEmbree();
    void buildScene();

    // rendering passes
    void renderDepthPass(float *depth_buffer);                                // trace rays, get distances
    void smoothDepthBuffer(float *depth_in, float *depth_out);                // blur depth for smoother look
    void smoothDepthBufferSeparable(float *depth_in, float *depth_out);       // faster blurring
    void renderFluidSurface(unsigned char *framebuffer, float *depth_buffer); // shade and output final image

    // helper functions
    glm::vec3 generateRayDirection(int pixel_x, int pixel_y);                      // get ray from camera through pixel
    glm::vec3 reconstructWorldPosition(int x, int y, float depth);                 // convert depth to 3d point
    glm::vec3 computeNormalFromDepth(int x, int y, float *depth_buffer);           // get surface normal
    glm::vec3 shadeFluid(const glm::vec3 &surface_point, const glm::vec3 &normal); // lighting

    // embree callbacks for sphere intersection testing
    static void sphereBoundsFunc(const struct RTCBoundsFunctionArguments *args);
    static void sphereIntersectFunc(const struct RTCIntersectFunctionNArguments *args);
    static void sphereOccludedFunc(const struct RTCOccludedFunctionNArguments *args);
};

#endif