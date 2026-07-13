cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Enable C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME firmware_stm32_control_unit)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and C++ languages
enable_language(C ASM CXX)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Create output directory for generated files
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/generated")

# Generate git constants header before build
find_package(Git QUIET)
if(GIT_FOUND)
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/generated/gitconstants.hh
        COMMAND ${CMAKE_COMMAND}
            -D GIT_EXECUTABLE=${GIT_EXECUTABLE}
            -D OUTPUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/generated
            -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_git_constants.cmake
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating git constants header..."
        VERBATIM
    )
    
    # Create a target that depends on the generated file
    add_custom_target(generate_git_constants ALL
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/generated/gitconstants.hh
    )
    
    # Make the executable depend on git constants generation
    add_dependencies(${CMAKE_PROJECT_NAME} generate_git_constants)
else()
    message(WARNING "Git not found, skipping git constants generation")
endif()
add_subdirectory(cmake/stm32cubemx)

# Shared libraries from the stm32_libs submodule (../stm32_libs) - out-of-tree
# add_subdirectory() needs an explicit binary dir.
add_subdirectory(../stm32_libs/common_stm32 ${CMAKE_BINARY_DIR}/stm32_libs/common_stm32)
add_subdirectory(../stm32_libs/tmc2209 ${CMAKE_BINARY_DIR}/stm32_libs/tmc2209)
add_subdirectory(../stm32_libs/usb_pd ${CMAKE_BINARY_DIR}/stm32_libs/usb_pd)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/app.cc
    # ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/sigmoid_stepper.cc
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_BINARY_DIR}/generated
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    STM32G4xx    
    STM32G4
)

# Propagate extra symbols to CubeMX interface (affects HAL sources, too)
target_compile_definitions(stm32cubemx INTERFACE STM32G4xx STM32G4)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    tmc2209
    common_stm32
    usb_pd

    # Add user defined libraries
)
