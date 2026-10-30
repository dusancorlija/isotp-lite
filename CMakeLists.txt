cmake_minimum_required(VERSION 3.16)
project(isotp-lite C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ISOTP_WERROR "Treat warnings as errors" OFF)
set(WARN -Wall -Wextra -Wpedantic -Wconversion -Wshadow)

add_library(isotp src/isotp.c)
target_include_directories(isotp PUBLIC include)
target_compile_options(isotp PRIVATE ${WARN} $<$<BOOL:${ISOTP_WERROR}>:-Werror>)

add_library(vbus sim/vbus.c)
target_include_directories(vbus PUBLIC sim)
target_link_libraries(vbus PUBLIC isotp)
target_compile_options(vbus PRIVATE ${WARN} $<$<BOOL:${ISOTP_WERROR}>:-Werror>)

enable_testing()
add_executable(test_isotp tests/test_isotp.c)
target_link_libraries(test_isotp vbus)
add_test(NAME test_isotp COMMAND test_isotp)

add_executable(throughput examples/throughput.c)
target_link_libraries(throughput vbus)

add_executable(uds_download examples/uds_download.c)
target_link_libraries(uds_download vbus)
add_executable(trace examples/trace.c)
target_link_libraries(trace vbus)
