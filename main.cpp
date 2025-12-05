// main.cpp

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <chrono>
#include "SPH.h"
#include "RayTracer.h"

// --- Global Variables and Constants ---
const int WINDOW_WIDTH = 640;
const int WINDOW_HEIGHT = 480;
SPHSolver *solver = nullptr;
RayTracer *raytracer = nullptr;

// Framebuffer for ray traced image
unsigned char *framebuffer = nullptr;

// OpenGL texture to display the ray traced image
GLuint texture_id = 0;

void framebuffer_size_callback(GLFWwindow *window, int width, int height)
{
    glViewport(0, 0, width, height);
}

void init_texture()
{
    // Create OpenGL texture to display ray traced image
    glGenTextures(1, &texture_id);
    glBindTexture(GL_TEXTURE_2D, texture_id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

void display_framebuffer()
{
    // Upload framebuffer to texture
    glBindTexture(GL_TEXTURE_2D, texture_id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, WINDOW_WIDTH, WINDOW_HEIGHT,
                 0, GL_RGB, GL_UNSIGNED_BYTE, framebuffer);

    // Draw fullscreen quad with the texture
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture_id);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, 1, 0, 1, -1, 1);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glBegin(GL_QUADS);
    glTexCoord2f(0, 1);
    glVertex2f(0, 0);
    glTexCoord2f(1, 1);
    glVertex2f(1, 0);
    glTexCoord2f(1, 0);
    glVertex2f(1, 1);
    glTexCoord2f(0, 0);
    glVertex2f(0, 1);
    glEnd();

    glDisable(GL_TEXTURE_2D);
}

void draw_container_overlay()
{
    // Draw wireframe box over the ray-traced image
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);

    // Setup projection for 3D wireframe
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glm::mat4 projection = glm::perspective(glm::radians(45.0f),
                                            (float)WINDOW_WIDTH / (float)WINDOW_HEIGHT, 0.1f, 100.0f);
    glLoadMatrixf(glm::value_ptr(projection));

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glm::mat4 view = glm::lookAt(
        glm::vec3(3.0f, 2.5f, 3.0f),
        glm::vec3(1.0f, 1.0f, 1.0f),
        glm::vec3(0.0f, 1.0f, 0.0f));
    glLoadMatrixf(glm::value_ptr(view));

    // Draw wireframe box
    float tank_size = 2.0f;
    glColor3f(0.7f, 0.7f, 0.7f);
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    // Floor and Ceiling
    for (int i = 0; i <= 1; ++i)
    {
        float y = i * tank_size;
        glVertex3f(0.0f, y, 0.0f);
        glVertex3f(tank_size, y, 0.0f);
        glVertex3f(tank_size, y, 0.0f);
        glVertex3f(tank_size, y, tank_size);
        glVertex3f(tank_size, y, tank_size);
        glVertex3f(0.0f, y, tank_size);
        glVertex3f(0.0f, y, tank_size);
        glVertex3f(0.0f, y, 0.0f);
    }
    // Vertical edges
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, tank_size, 0.0f);
    glVertex3f(tank_size, 0.0f, 0.0f);
    glVertex3f(tank_size, tank_size, 0.0f);
    glVertex3f(tank_size, 0.0f, tank_size);
    glVertex3f(tank_size, tank_size, tank_size);
    glVertex3f(0.0f, 0.0f, tank_size);
    glVertex3f(0.0f, tank_size, tank_size);
    glEnd();

    glEnable(GL_DEPTH_TEST);
}

// --- Main Function ---

int main()
{
    // Initialize GLFW
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    GLFWwindow *window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "SPH Ray Tracer", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        std::cerr << "Failed to create GLFW window" << std::endl;
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // Initialize GLEW
    if (glewInit() != GLEW_OK)
    {
        std::cerr << "Failed to initialize GLEW" << std::endl;
        return -1;
    }

    // Enable depth test
    glEnable(GL_DEPTH_TEST);

    // --- Initialize SPH Simulation ---
    solver = new SPHSolver(1000);

    // --- Initialize Ray Tracer ---
    raytracer = new RayTracer(WINDOW_WIDTH, WINDOW_HEIGHT);

    // Set up camera
    raytracer->setCamera(
        glm::vec3(3.0f, 2.5f, 3.0f),
        glm::vec3(1.0f, 1.0f, 1.0f),
        glm::vec3(0.0f, 1.0f, 0.0f),
        45.0f);

    // Allocate framebuffer
    framebuffer = new unsigned char[WINDOW_WIDTH * WINDOW_HEIGHT * 3];

    // Initialize texture
    init_texture();

    std::cout << "\n=== Ray Tracer Ready ===" << std::endl;
    std::cout << "Resolution: " << WINDOW_WIDTH << "x" << WINDOW_HEIGHT << std::endl;
    std::cout << "Starting render loop...\n"
              << std::endl;

    // Main render loop
    int frame_count = 0;
    auto last_time = std::chrono::high_resolution_clock::now();

    while (!glfwWindowShouldClose(window))
    {
        // --- 1. Update SPH Physics ---
        solver->update();

        // --- 2. Get particles ---
        const auto &particles = solver->getParticles();

        // --- 3. Only ray trace every 2 frames ---
        if (frame_count % 2 == 0)
        {
            std::vector<glm::vec3> positions;
            positions.reserve(particles.size());
            for (const auto &p : particles)
            {
                positions.push_back(p.pos);
            }

            raytracer->updateScene(positions, 0.025f);
            raytracer->render(framebuffer);
        }

        // --- 4. Display result ---
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        display_framebuffer();
        draw_container_overlay();

        glfwSwapBuffers(window);
        glfwPollEvents();

        // --- 5. Timing Info ---
        frame_count++;
        if (frame_count % 30 == 0)
        {
            auto current_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time);
            float fps = 30000.0f / duration.count();
            std::cout << "FPS: " << fps << " | Particles: " << particles.size() << std::endl;
            last_time = current_time;
        }
    }

    // Cleanup
    delete[] framebuffer;
    delete raytracer;
    delete solver;
    glfwTerminate();

    std::cout << "\nCleanup complete. Exiting..." << std::endl;
    return 0;
}