# CMakeLists.txt

cmake_minimum_required(VERSION 3.10)
project(SPH_RayTracer_Project CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# --- VCPKG INTEGRATION ---
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../vcpkg/scripts/buildsystems/vcpkg.cmake")
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../vcpkg/scripts/buildsystems/vcpkg.cmake")
elseif(EXISTS "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
    set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
endif()

# --- FIND EXTERNAL LIBRARIES ---
find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glm REQUIRED)
find_package(GLEW REQUIRED)  # Add this line
find_package(Embree REQUIRED)

# --- APPLICATION SETUP ---
add_executable(sph_app main.cpp SPH.cpp)

# Link all libraries to the executable
target_link_libraries(sph_app
    glfw
    OpenGL::GL
    GLEW::GLEW  # Add this line
    embree
)