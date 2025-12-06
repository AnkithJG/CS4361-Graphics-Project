#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <chrono>
#include "SPH.h"
#include "RayTracer.h"

const int WINDOW_WIDTH = 640;
const int WINDOW_HEIGHT = 480;

// global pointers to main simulation objects
SPHSolver *solver = nullptr;
RayTracer *raytracer = nullptr;

// framebuffer to render to (rgb, unsigned byte)
unsigned char *framebuffer = nullptr;

// opengl texture id for displaying the rendered image
GLuint texture_id = 0;

// callback for when window resizes
void framebuffer_size_callback(GLFWwindow *window, int width, int height)
{
    glViewport(0, 0, width, height);
}

// create and configure opengl texture
void init_texture()
{
    glGenTextures(1, &texture_id);
    glBindTexture(GL_TEXTURE_2D, texture_id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

// display the framebuffer on screen
void display_framebuffer()
{
    // upload framebuffer to texture
    glBindTexture(GL_TEXTURE_2D, texture_id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, WINDOW_WIDTH, WINDOW_HEIGHT,
                 0, GL_RGB, GL_UNSIGNED_BYTE, framebuffer);

    // set up for texture rendering
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture_id);

    // ortho projection for fullscreen quad
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, 1, 0, 1, -1, 1);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // draw fullscreen textured quad
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

// draw container wireframe on top of rendered image
void draw_container_overlay()
{
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);

    // 3d perspective projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glm::mat4 projection = glm::perspective(glm::radians(45.0f),
                                            (float)WINDOW_WIDTH / (float)WINDOW_HEIGHT, 0.1f, 100.0f);
    glLoadMatrixf(glm::value_ptr(projection));

    // camera looking at tank
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glm::mat4 view = glm::lookAt(
        glm::vec3(3.0f, 2.5f, 3.0f),
        glm::vec3(1.0f, 1.0f, 1.0f),
        glm::vec3(0.0f, 1.0f, 0.0f));
    glLoadMatrixf(glm::value_ptr(view));

    // draw cube wireframe
    float tank_size = 2.0f;
    glColor3f(0.7f, 0.7f, 0.7f);
    glLineWidth(2.0f);
    glBegin(GL_LINES);
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

// main program
int main()
{
    // init glfw window system
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    // create window
    GLFWwindow *window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "SPH Ray Tracer", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        std::cerr << "Failed to create GLFW window" << std::endl;
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // init opengl extensions
    if (glewInit() != GLEW_OK)
    {
        std::cerr << "Failed to initialize GLEW" << std::endl;
        return -1;
    }

    glEnable(GL_DEPTH_TEST);

    // create simulation objects
    solver = new SPHSolver(1000); // 1000 particles

    raytracer = new RayTracer(WINDOW_WIDTH, WINDOW_HEIGHT);

    // set up camera
    raytracer->setCamera(
        glm::vec3(3.0f, 2.5f, 3.0f),
        glm::vec3(1.0f, 1.0f, 1.0f),
        glm::vec3(0.0f, 1.0f, 0.0f),
        45.0f);

    // allocate framebuffer for rendering
    framebuffer = new unsigned char[WINDOW_WIDTH * WINDOW_HEIGHT * 3];

    // set up texture
    init_texture();

    std::cout << "\n=== Ray Tracer Ready ===" << std::endl;
    std::cout << "Resolution: " << WINDOW_WIDTH << "x" << WINDOW_HEIGHT << std::endl;
    std::cout << "Starting render loop...\n"
              << std::endl;

    // render loop
    int frame_count = 0;
    auto last_time = std::chrono::high_resolution_clock::now();

    while (!glfwWindowShouldClose(window))
    {
        // update physics
        solver->update();

        const auto &particles = solver->getParticles();

        // render every 3 frames (so sph can simulate ahead)
        if (frame_count % 3 == 0)
        {
            // extract particle positions
            std::vector<glm::vec3> positions;
            positions.reserve(particles.size());
            for (const auto &p : particles)
            {
                positions.push_back(p.pos);
            }

            // render particles
            raytracer->updateScene(positions, 0.025f);
            raytracer->render(framebuffer);
        }

        // clear and draw
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        display_framebuffer();
        draw_container_overlay();

        glfwSwapBuffers(window);
        glfwPollEvents();

        frame_count++;

        // print fps every 30 frames
        if (frame_count % 30 == 0)
        {
            auto current_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time);
            float fps = 30000.0f / duration.count();
            std::cout << "FPS: " << fps << " | Particles: " << particles.size() << std::endl;
            last_time = current_time;
        }
    }

    // cleanup
    delete[] framebuffer;
    delete raytracer;
    delete solver;
    glfwTerminate();

    std::cout << "\nCleanup complete. Exiting..." << std::endl;
    return 0;
}