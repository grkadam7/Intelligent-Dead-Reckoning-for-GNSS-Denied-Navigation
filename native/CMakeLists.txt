cmake_minimum_required(VERSION 3.14)
project(idr_mobile VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

# Shared library for mobile platforms (Android .so, iOS dylib/framework)
add_library(idr_mobile SHARED
    src/idr_mobile.cpp
)

target_include_directories(idr_mobile PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if (BUILD_TESTING)
    enable_testing()
    add_executable(test_abi
        tests/test_abi.cpp
    )
    target_link_libraries(test_abi PRIVATE idr_mobile)
    add_test(NAME abi_tests COMMAND test_abi)
endif()

