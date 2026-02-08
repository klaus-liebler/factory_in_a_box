#!/usr/bin/env cmake -P
#
# Script to generate git constants header file
# Usage: cmake -P generate_git_constants.cmake
#

# Find git executable
find_package(Git REQUIRED)

# Get git information
execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
    OUTPUT_VARIABLE GIT_COMMIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
    OUTPUT_VARIABLE GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --tags --always
    OUTPUT_VARIABLE GIT_TAG
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

execute_process(
    COMMAND ${GIT_EXECUTABLE} log -1 --format=%ai
    OUTPUT_VARIABLE GIT_COMMIT_DATE
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

execute_process(
    COMMAND ${GIT_EXECUTABLE} log -1 --format=%an
    OUTPUT_VARIABLE GIT_COMMIT_AUTHOR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

execute_process(
    COMMAND ${GIT_EXECUTABLE} log -1 --format=%s
    OUTPUT_VARIABLE GIT_COMMIT_MESSAGE
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

# Check if working directory is dirty
execute_process(
    COMMAND ${GIT_EXECUTABLE} status --porcelain
    OUTPUT_VARIABLE GIT_STATUS_OUTPUT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

if(GIT_STATUS_OUTPUT)
    set(GIT_DIRTY "true")
else()
    set(GIT_DIRTY "false")
endif()

# Get current timestamp
string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d %H:%M:%S")

# Create output directory
file(MAKE_DIRECTORY "${OUTPUT_DIR}")

# Generate header file
file(WRITE "${OUTPUT_DIR}/gitconstants.hh"
"#pragma once

/**
 * @file gitconstants.hh
 * @brief Auto-generated Git information constants
 * Generated at: ${BUILD_TIMESTAMP}
 */

#include <string_view>

namespace git {

/// Git commit short hash
constexpr std::string_view COMMIT_HASH = \"${GIT_COMMIT_HASH}\";

/// Git branch name
constexpr std::string_view BRANCH = \"${GIT_BRANCH}\";

/// Git tag or commit hash
constexpr std::string_view TAG = \"${GIT_TAG}\";

/// Last commit date and time
constexpr std::string_view COMMIT_DATE = \"${GIT_COMMIT_DATE}\";

/// Last commit author
constexpr std::string_view COMMIT_AUTHOR = \"${GIT_COMMIT_AUTHOR}\";

/// Last commit message
constexpr std::string_view COMMIT_MESSAGE = \"${GIT_COMMIT_MESSAGE}\";

/// Is the working directory dirty (has uncommitted changes)?
constexpr bool IS_DIRTY = ${GIT_DIRTY};

/// Build timestamp
constexpr std::string_view BUILD_TIMESTAMP = \"${BUILD_TIMESTAMP}\";

/// Full version string
constexpr std::string_view VERSION = \"${GIT_TAG}-${GIT_COMMIT_HASH}\";

} // namespace git
")

message(STATUS "Generated git constants header: ${OUTPUT_DIR}/gitconstants.hh")
message(STATUS "  Commit: ${GIT_COMMIT_HASH}")
message(STATUS "  Branch: ${GIT_BRANCH}")
message(STATUS "  Tag: ${GIT_TAG}")
message(STATUS "  Dirty: ${GIT_DIRTY}")
