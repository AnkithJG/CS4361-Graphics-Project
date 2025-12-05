cmake_minimum_required(VERSION 3.10)

# --- VCPKG INTEGRATION (MUST BE BEFORE project()) ---
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../vcpkg/scripts/buildsystems/vcpkg.cmake")
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
elseif(DEFINED ENV{VCPKG_ROOT})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
endif()

project(SPH_RayTracer_Project CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# --- FIND EXTERNAL LIBRARIES ---
find_package(OpenMP REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glm REQUIRED)
find_package(GLEW REQUIRED)
find_package(embree 4 REQUIRED)  # Use lowercase 'embree' with version 4

# --- APPLICATION SETUP ---
add_executable(sph_app main.cpp SPH.cpp RayTracer.cpp)

# Link all libraries
target_link_libraries(sph_app
    glfw
    OpenGL::GL
    GLEW::GLEW
    embree
    OpenMP::OpenMP_CXX
)