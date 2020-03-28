cmake_minimum_required (VERSION 3.0)


set(CMAKE_C_COMPILER /usr/bin/clang-9)
set(CMAKE_CXX_COMPILER /usr/bin/clang++-9)
set(CMAKE_BUILD_TYPE Debug)

project(catnav2)

# set (CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(common "-fno-omit-frame-pointer -pg -std=c++17")
set(profiling "-fno-omit-frame-pointer -pg")
set(warnings "-Wall \
  -Wextra \
  -Wshadow \
  -Wnon-virtual-dtor \
  -Wold-style-cast \
  -Wpedantic \
  -Wcast-align \
  -Wunused \
  -Woverloaded-virtual \
  -Wconversion \
  -Wsign-conversion \
  -Wnull-dereference \
  -Wdouble-promotion \
  -Wformat=2")
# For later:
# -Werror
# -Wduplicated-cond \
# -Wduplicated-branches \
# -Wlogical-op \
# -Wuseless-cast \
# -Wlifetime -> coming in later version of clang.
set(optimizations_debug "-O2")
set(optimizations_release "-O3 -flto -ffast-math")
set(debug "-g")
set(perf_bench "-fno-omit-frame-pointer")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} ${common} ${debug} ${warnings} ${optimizations_debug}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${common} ${optimizations_release}")

# for clang++ use -fuse-ld=gold to avoid LD error "File format not recognized"
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  # message("Clang detected")
  if(CMAKE_BUILD_TYPE MATCHES Debug)
    # message("Using clang sanitizers")
    # set(sanitizer "-fsanitize=address")
    # set(sanitizer "-fsanitize=memory")
    # set(sanitizer "-fsanitize=memory -fsanitize-memory-track-origins")
    # set(sanitizer "-fsanitize=thread")
    # set(CMAKE_EXE_LINKER_FLAGS "-fuse-ld=gold")
    # set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${sanitizer}")
  endif(CMAKE_BUILD_TYPE MATCHES Debug)
endif()

# find_package(Eigen3 REQUIRED)
# include_directories(${EIGEN3_INCLUDE_DIR})

# find_package(Range-V3 REQUIRED)
# include_directories(${RANGE-V3_INCLUDE_DIR})

# find_package(Hana REQUIRED)
# include_directories(${HANA_INCLUDE_DIR})

# find_package(Boost REQUIRED)
# include_directories(${Boost_INCLUDE_DIRS})

enable_testing()
add_subdirectory(lib)
add_subdirectory(test)
add_subdirectory(src)

if( EXISTS "${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json" )
  execute_process( COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json
    ${CMAKE_CURRENT_SOURCE_DIR}/compile_commands.json
  )
endif()
