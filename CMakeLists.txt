#---------------------------------------------------------------------------------------------------
# CMake Build Configuration for the Ray Tracing Weekend Series
#---------------------------------------------------------------------------------------------------

cmake_minimum_required ( VERSION 3.1.0...3.27.0 )

project ( RTWeekend LANGUAGES CXX )

# Set to C++14
set ( CMAKE_CXX_STANDARD          14 )
set ( CMAKE_CXX_STANDARD_REQUIRED ON )
set ( CMAKE_CXX_EXTENSIONS        ON )

# Source

set ( EXTERNAL
  src/external/stb_image.h
)

set ( SOURCE_NEXT_WEEK
  src/TheNextWeek/main.cc
  src/TheNextWeek/aabb.h
  src/TheNextWeek/bvh.h
  src/TheNextWeek/camera.h
  src/TheNextWeek/color.h
  src/TheNextWeek/constant_medium.h
  src/TheNextWeek/hittable.h
  src/TheNextWeek/hittable_list.h
  src/TheNextWeek/interval.h
  src/TheNextWeek/material.h
  src/TheNextWeek/perlin.h
  src/TheNextWeek/quad.h
  src/TheNextWeek/ray.h
  src/TheNextWeek/rtw_stb_image.h
  src/TheNextWeek/rtweekend.h
  src/TheNextWeek/sphere.h
  src/TheNextWeek/texture.h
  src/TheNextWeek/vec3.h
  src/TheNextWeek/triangle.h

)

include_directories(src)

# Compiler-specific flags
message (STATUS "Compiler ID: " ${CMAKE_CXX_COMPILER_ID})
message (STATUS "Release flags: " ${CMAKE_CXX_FLAGS_RELEASE})
message (STATUS "Debug flags: " ${CMAKE_CXX_FLAGS_DEBUG})

if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    add_compile_options("/W4")
    add_compile_options("/we 4265")
    add_compile_options("/we 5204")
    add_compile_options("/wd 4100")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wnon-virtual-dtor)
    add_compile_options(-Wreorder)
    add_compile_options(-Wmaybe-uninitialized)
    add_compile_options(-Wunused-variable)
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wnon-virtual-dtor)
    add_compile_options(-Wreorder)
    add_compile_options(-Wsometimes-uninitialized)
    add_compile_options(-Wunused-variable)
endif()

# Executables
add_executable(theNextWeek ${EXTERNAL} ${SOURCE_NEXT_WEEK})
