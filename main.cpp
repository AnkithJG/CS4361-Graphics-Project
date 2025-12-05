// main.cpp

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include "SPH.h"

// --- Global Variables and Constants ---
const int WINDOW_WIDTH = 1000;
const int WINDOW_HEIGHT = 1000;
SPHSolver *solver = nullptr;

// Simple Camera Setup
glm::vec3 cameraPos = glm::vec3(3.0f, 2.5f, 3.0f);    // Position
glm::vec3 cameraTarget = glm::vec3(1.0f, 1.0f, 1.0f); // Looking at center of the tank
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

// --- Drawing Functions ---


   void render_particles()
{
    float tank_size = 2.0f;

    // Faint “glass” tank
    glColor4f(0.7f, 0.7f, 0.7f, 0.15f);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    // bottom
    glVertex3f(0.0f, 0.0f, 0.0f);       glVertex3f(tank_size, 0.0f, 0.0f);
    glVertex3f(tank_size, 0.0f, 0.0f);  glVertex3f(tank_size, 0.0f, tank_size);
    glVertex3f(tank_size, 0.0f, tank_size); glVertex3f(0.0f, 0.0f, tank_size);
    glVertex3f(0.0f, 0.0f, tank_size);  glVertex3f(0.0f, 0.0f, 0.0f);
    // vertical
    glVertex3f(0.0f, 0.0f, 0.0f);       glVertex3f(0.0f, tank_size, 0.0f);
    glVertex3f(tank_size, 0.0f, 0.0f);  glVertex3f(tank_size, tank_size, 0.0f);
    glVertex3f(tank_size, 0.0f, tank_size); glVertex3f(tank_size, tank_size, tank_size);
    glVertex3f(0.0f, 0.0f, tank_size);  glVertex3f(0.0f, tank_size, tank_size);
    // top
    glVertex3f(0.0f, tank_size, 0.0f);      glVertex3f(tank_size, tank_size, 0.0f);
    glVertex3f(tank_size, tank_size, 0.0f); glVertex3f(tank_size, tank_size, tank_size);
    glVertex3f(tank_size, tank_size, tank_size); glVertex3f(0.0f, tank_size, tank_size);
    glVertex3f(0.0f, tank_size, tank_size); glVertex3f(0.0f, tank_size, 0.0f);
    glEnd();

    // Water particles
    const auto &particles = solver->getParticles();

    glPointSize(10.0f);

    glBegin(GL_POINTS);
    for (const auto &p : particles)
    {
        float t = p.pos.y / tank_size;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;

        float r = 0.05f * (1.0f - t);
        float g = 0.35f + 0.15f * t;
        float b = 0.9f  + 0.1f  * t;
        float a = 0.85f;

        glColor4f(r, g, b, a);
        glVertex3f(p.pos.x, p.pos.y, p.pos.z);
    }
    glEnd();
}



void framebuffer_size_callback(GLFWwindow *window, int width, int height)
{
    glViewport(0, 0, width, height);
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

    // Requesting a modern OpenGL context

    GLFWwindow *window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "3D SPH Simulation Core", NULL, NULL);
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

    // Enable 3D features
    glEnable(GL_DEPTH_TEST);

    // --- SPH Initialization ---
    // Using 3000 particles as planned
    solver = new SPHSolver(500);

    // Main render loop
    while (!glfwWindowShouldClose(window))
    {
        // --- Simulation Update ---
        solver->update();

        // Dark, slightly bluish background so the water stands out
        // Dark, slightly bluish background
        glClearColor(0.02f, 0.02f, 0.08f, 1.0f);


        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Enable alpha blending for translucent “water drops”
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // Allow variable point sizes
        glEnable(GL_PROGRAM_POINT_SIZE);
// ---- CAMERA ----
        glm::vec3 cameraPos    = glm::vec3(3.5f, 3.0f, 3.5f);
        glm::vec3 cameraTarget = glm::vec3(1.0f, 1.0f, 1.0f);
        glm::vec3 cameraUp     = glm::vec3(0.0f, 1.0f, 0.0f);

        // Build view + projection matrices
        glm::mat4 view = glm::lookAt(cameraPos, cameraTarget, cameraUp);
        glm::mat4 proj = glm::perspective(glm::radians(45.0f), (float)WINDOW_WIDTH / (float)WINDOW_HEIGHT, 0.1f, 100.0f);
        // Convert glm → OpenGL
        glMatrixMode(GL_PROJECTION);
        glLoadMatrixf(glm::value_ptr(proj));

        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixf(glm::value_ptr(view));

        render_particles();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup
    delete solver;
    glfwTerminate();
    return 0;
}