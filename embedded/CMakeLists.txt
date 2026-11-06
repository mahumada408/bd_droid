# Cross-build of the whole-body controller core for the STM32H743 (Cortex-M7,
# 480 MHz, double-precision FPU) on the NUCLEO-H743ZI2.
#
#   embedded/build.sh            # configure + build, prints flash / RAM use
#
# Builds, from source:
#   blasfeo  (TARGET=GENERIC: plain C kernels)   -> libblasfeo.a
#   hpipm    (TARGET=GENERIC)                    -> libhpipm.a
#   wbc_core (src/wbc: rbd.cpp, wbc_core.cpp)    -> libwbc_core.a
#   wbc_h743.elf: bare-metal image (startup, linker script, no HAL) that
#     initialises the controller from wbc_default_params.h and times wbc steps
#     with the DWT cycle counter. Flashable, but it runs on the reset clock
#     (64 MHz HSI) until clock setup is added.
cmake_minimum_required(VERSION 3.16)
project(wbc_h743 C CXX ASM)

set(ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

# BLASFEO / HPIPM sources: the copies acados ships (the same versions the
# simulator links), or a fresh checkout of the same commits.
set(BLASFEO_COMMIT 9c2a8345f9de37fbb4b3cd84810b0250810de622)
set(HPIPM_COMMIT 002438be7babe62e99a6ead55474ce435b3e59a2)
set(ACADOS_EXTERNAL "$ENV{HOME}/software/acados/external" CACHE PATH "acados/external (blasfeo, hpipm sources)")
if(NOT BLASFEO_SRC AND EXISTS ${ACADOS_EXTERNAL}/blasfeo/CMakeLists.txt)
  set(BLASFEO_SRC ${ACADOS_EXTERNAL}/blasfeo)
endif()
if(NOT HPIPM_SRC AND EXISTS ${ACADOS_EXTERNAL}/hpipm/CMakeLists.txt)
  set(HPIPM_SRC ${ACADOS_EXTERNAL}/hpipm)
endif()
include(FetchContent)
if(NOT BLASFEO_SRC)
  FetchContent_Populate(blasfeo GIT_REPOSITORY https://github.com/giaf/blasfeo.git GIT_TAG ${BLASFEO_COMMIT})
  set(BLASFEO_SRC ${blasfeo_SOURCE_DIR})
endif()
if(NOT HPIPM_SRC)
  FetchContent_Populate(hpipm GIT_REPOSITORY https://github.com/giaf/hpipm.git GIT_TAG ${HPIPM_COMMIT})
  set(HPIPM_SRC ${hpipm_SOURCE_DIR})
endif()
message(STATUS "blasfeo: ${BLASFEO_SRC}")
message(STATUS "hpipm:   ${HPIPM_SRC}")

# ---- blasfeo (generic C, no external dependencies: no malloc / printf)
set(TARGET GENERIC CACHE STRING "" FORCE)
set(LA HIGH_PERFORMANCE CACHE STRING "" FORCE)
set(EXT_DEP OFF CACHE BOOL "" FORCE)
set(BLAS_API OFF CACHE BOOL "" FORCE)
set(BLASFEO_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BLASFEO_TESTING OFF CACHE BOOL "" FORCE)
set(BLASFEO_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
add_subdirectory(${BLASFEO_SRC} blasfeo EXCLUDE_FROM_ALL)

# ---- hpipm (uses the blasfeo target above)
set(HPIPM_TESTING OFF CACHE BOOL "" FORCE)
set(HPIPM_FIND_BLASFEO OFF CACHE BOOL "" FORCE)
set(BLASFEO_PATH ${BLASFEO_SRC} CACHE STRING "" FORCE)
set(BLASFEO_INCLUDE_DIR ${BLASFEO_SRC}/include CACHE STRING "" FORCE)
add_subdirectory(${HPIPM_SRC} hpipm EXCLUDE_FROM_ALL)
target_include_directories(hpipm PUBLIC ${HPIPM_SRC}/include ${BLASFEO_SRC}/include)

# ---- controller core
add_library(wbc_core STATIC ${ROOT}/src/wbc/rbd.cpp ${ROOT}/src/wbc/wbc_core.cpp)
target_include_directories(wbc_core PUBLIC ${ROOT}/src/wbc)
target_compile_features(wbc_core PUBLIC cxx_std_17)
target_compile_options(wbc_core PRIVATE -O2 -Wall -Wextra -Wno-unused-parameter)
target_link_libraries(wbc_core PUBLIC hpipm blasfeo)

# ---- bare-metal image
add_executable(wbc_h743.elf
  stm32h743/startup.c
  stm32h743/main.cpp)
target_link_libraries(wbc_h743.elf PRIVATE wbc_core m)
target_compile_options(wbc_h743.elf PRIVATE -O2 -Wall)
target_link_options(wbc_h743.elf PRIVATE
  -T${CMAKE_CURRENT_SOURCE_DIR}/stm32h743/stm32h743.ld
  -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/wbc_h743.map
  -Wl,--print-memory-usage
  -Wl,--no-warn-rwx-segments)
add_custom_command(TARGET wbc_h743.elf POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:wbc_h743.elf> ${CMAKE_CURRENT_BINARY_DIR}/wbc_h743.bin
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:wbc_h743.elf>)
