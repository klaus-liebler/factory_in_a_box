/**
 * @file gitconstants_example.cc
 * @brief Example of how to use git constants in your firmware
 */

#include "gitconstants.hh"
#include "log.h"
#include <cstdio>

void print_version_info() {
    // Print all available git information
    printf("\n=== Application Version Information ===\n");
    printf("Version String:  %.*s\n", (int)git::VERSION.length(), git::VERSION.data());
    printf("Commit Hash:     %.*s\n", (int)git::COMMIT_HASH.length(), git::COMMIT_HASH.data());
    printf("Branch:          %.*s\n", (int)git::BRANCH.length(), git::BRANCH.data());
    printf("Tag:             %.*s\n", (int)git::TAG.length(), git::TAG.data());
    printf("Commit Date:     %.*s\n", (int)git::COMMIT_DATE.length(), git::COMMIT_DATE.data());
    printf("Commit Author:   %.*s\n", (int)git::COMMIT_AUTHOR.length(), git::COMMIT_AUTHOR.data());
    printf("Commit Message:  %.*s\n", (int)git::COMMIT_MESSAGE.length(), git::COMMIT_MESSAGE.data());
    printf("Build Timestamp: %.*s\n", (int)git::BUILD_TIMESTAMP.length(), git::BUILD_TIMESTAMP.data());
    printf("Dirty Flag:      %s\n", git::IS_DIRTY ? "YES (uncommitted changes)" : "NO (clean)");
    printf("======================================\n\n");
}

// Example: Send version info over UART for remote diagnostic
void send_version_via_uart() {
    log_info("=== Firmware Version Information ===");
    log_info("Build Time: %.*s", (int)git::BUILD_TIMESTAMP.length(), git::BUILD_TIMESTAMP.data());
    log_info("Commit: %.*s on %.*s", 
             (int)git::COMMIT_HASH.length(), git::COMMIT_HASH.data(),
             (int)git::BRANCH.length(), git::BRANCH.data());
    log_info("Version: %.*s", (int)git::VERSION.length(), git::VERSION.data());
    
    if (git::IS_DIRTY) {
        log_warning("⚠ WARNING: Built from dirty repository!");
    }
}

// Example: Check if firmware was built from a specific branch
bool is_built_from_main_branch() {
    return git::BRANCH == std::string_view("main") || 
           git::BRANCH == std::string_view("master");
}


