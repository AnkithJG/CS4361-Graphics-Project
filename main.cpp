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
    if (!solver)
        return;

    // Setup Model-View-Projection (MVP) matrices
    glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)WINDOW_WIDTH / (float)WINDOW_HEIGHT, 0.1f, 100.0f);
    glm::mat4 view = glm::lookAt(cameraPos, cameraTarget, cameraUp);

    // Set OpenGL matrices
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(glm::value_ptr(projection));
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(glm::value_ptr(view));

    // Simple 3D Container (wireframe box for context)
    float tank_size = 2.0f;
    glColor3f(0.5f, 0.5f, 0.5f);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    // Floor and Ceiling
    for (int i = 0; i <= 1; ++i)
    { // i=0 for floor, i=1 for ceiling
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

    // Draw Particles (OpenGL Point Sprites)
    const auto &particles = solver->getParticles();

    glColor3f(1.0f, 1.0f, 1.0f); // White color
    glPointSize(8.0f);           // Particle size

    glBegin(GL_POINTS);
    for (const auto &p : particles)
    {
        // The position is already in world space, OpenGL handles the projection
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
    solver = new SPHSolver(2000);

    // Main render loop
    while (!glfwWindowShouldClose(window))
    {
        // --- Simulation Update ---
        solver->update();

        // --- Rendering ---
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Black background
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        render_particles();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup
    delete solver;
    glfwTerminate();
    return 0;
}