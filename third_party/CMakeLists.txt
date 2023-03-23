cmake_minimum_required(VERSION 2.8.9)

# Disable warnings for third-party code
add_compile_options(-w)

set(TARGET GENERIC CACHE STRING "Set CPU architecture target")
set(BLASFEO_EXAMPLES OFF CACHE BOOL "Examples enabled")
set(BLASFEO_TESTING OFF CACHE BOOL "Tests disabled")
set(HPIPM_TESTING OFF CACHE BOOL "Tests enabled")

add_subdirectory(blasfeo)
add_subdirectory(hpipm)

