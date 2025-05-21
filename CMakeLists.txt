cmake_minimum_required(VERSION 3.20..4.0)

if (SKBUILD)
    set(CR_KNOWLEDGE_EXTRACTION_VERSION ${SKBUILD_PROJECT_VERSION})
else ()
    set(CR_KNOWLEDGE_EXTRACTION_VERSION 0.1.0)
endif ()

project(CommonRoadKnowledgeExtraction
        LANGUAGES CXX
        VERSION ${CR_KNOWLEDGE_EXTRACTION_VERSION}
        DESCRIPTION "C++ extension for the commonroad-rule-simplification Python package")

set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

# CMP0126 (3.21) - Removal of normal variables by set(CACHE)
if (POLICY CMP0126)
    cmake_policy(SET CMP0126 NEW)
endif ()

# CMP0135 - URL download timestamp
if (POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif ()


# Adapted from Eigen3 - snippet to get a value for PROJECT_IS_TOP_LEVEL
# on CMake versions before v3.21.0
if (CMAKE_VERSION VERSION_LESS 3.21.0)
    if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
        set(PROJECT_IS_TOP_LEVEL ON)
    else ()
        set(PROJECT_IS_TOP_LEVEL OFF)
    endif ()
    set(${PROJECT_NAME}_IS_TOP_LEVEL ${PROJECT_IS_TOP_LEVEL})
endif ()

if (NOT SKBUILD)
    set(CMAKE_VERIFY_INTERFACE_HEADER_SETS ON)
endif ()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_COLOR_DIAGNOSTICS ON)

set(CMAKE_LINK_LIBRARIES_ONLY_TARGETS ON)

set(CMAKE_MESSAGE_CONTEXT_SHOW ON)

option(CR_KNOWLEDGE_EXTRACTION_BUILD_PYTHON_BINDINGS
        "Build Python bindings for CommonRoad Knowledge Extraction"
        OFF)

if (CR_KNOWLEDGE_EXTRACTION_BUILD_PYTHON_BINDINGS)
    find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)

    execute_process(
            COMMAND "${PYTHON_EXECUTABLE}" -m nanobind --cmake_dir
            OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE NB_DIR)
    list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
    find_package(nanobind CONFIG REQUIRED)

    if (SKBUILD)
        # SKBUILD_SELF_CONTAINED controls whether we try to build all dependencies
        # ourselves. This is used in order to build cross-platform wheels
        # using cibuildwheel.
        # We don't enable this in normal Python builds since it will generally
        # just slow down the build.
        set(SKBUILD_SELF_CONTAINED OFF)

        message(STATUS "PYTHON MODE - assuming we are invoked by pip/setup.py")
        message(STATUS "PYTHON MODE - building static libraries")

        if (DEFINED ENV{CIBUILDWHEEL})
            set(SKBUILD_SELF_CONTAINED ON)
        endif ()

        set(FETCHCONTENT_QUIET ON)

        # Globally build static libraries (affects all calls to add_library
        # without an explicit library type)
        set(BUILD_SHARED_LIBS OFF)

        set(CMAKE_POSITION_INDEPENDENT_CODE ON)
    else ()
        message(STATUS "PYTHON MODE - adding Python interface for compilation only")
    endif (SKBUILD)
endif (CR_KNOWLEDGE_EXTRACTION_BUILD_PYTHON_BINDINGS)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

# Ensure executables are in the top level directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

include(CMakeDependentOption)

set(CR_KNOWLEDGE_EXTRACTION_BUILD_EXTRAS_DEFAULT ON)
if (NOT CommonRoadKnowledgeExtraction_IS_TOP_LEVEL)
    set(CR_KNOWLEDGE_EXTRACTION_BUILD_EXTRAS_DEFAULT OFF)
endif ()

# TODO Maybe involve BUILD_TESTING here?
cmake_dependent_option(CR_KNOWLEDGE_EXTRACTION_BUILD_TESTS
        "Build tests"
        ${CR_KNOWLEDGE_EXTRACTION_BUILD_EXTRAS_DEFAULT}
        "NOT SKBUILD"
        OFF)

# add_library (without STATIC or SHARED) will respect BUILD_SHARED_LIBS when determining the library type
# The following definitions allow setting BUILD_SHARED_LIBS globally or only for this project
# (useful when including it into another project).
#
# If ${PROJECT_NAME}_BUILD_SHARED_LIBS is set to a specific value by another project or the user,
# then we will use that value (by setting BUILD_SHARED_LIBS to the value of ${PROJECT_NAME}_BUILD_SHARED_LIBS).
# Otherwise, the default for ${PROJECT_NAME}_BUILD_SHARED_LIBS is taken from BUILD_SHARED_LIBS if it exists.

if (DEFINED BUILD_SHARED_LIBS)
    set(_BUILD_SHARED_LIBS_DEFAULT ${BUILD_SHARED_LIBS})
else ()
    set(_BUILD_SHARED_LIBS_DEFAULT ON)
endif ()

option(${PROJECT_NAME}_BUILD_SHARED_LIBS "Build ${PROJECT_NAME} as a shared library" ${_BUILD_SHARED_LIBS_DEFAULT})

set(BUILD_SHARED_LIBS ${${PROJECT_NAME}_BUILD_SHARED_LIBS})

set(CMAKE_SUPPORTS_TRY_FIND_PACKAGE OFF)
if (CMAKE_VERSION VERSION_GREATER_EQUAL 3.24.0)
    set(CMAKE_SUPPORTS_TRY_FIND_PACKAGE ON)
else ()
    message(WARNING "Your CMake version (${CMAKE_VERSION}) does not support "
            "the FetchContent find_package integration introduced in CMake 3.24. "
            "As a fallback, we will simply build all dependencies ourselves "
            "irrespective of whether a suitable system version exists. "
            "While this does not impair functionality, it might slow down the build "
            "process a bit.\n"
            "In case you have all required dependencies installed, you can try "
            "enabling the option\n"
            "\tCR_KNOWLEDGE_EXTRACTION_SYSTEM_PACKAGES_FORCE\n"
            "which will force using find_package for all dependencies.")
endif ()

option(CR_KNOWLEDGE_EXTRACTION_SYSTEM_PACKAGES "Try to use system packages for dependencies" ON)
cmake_dependent_option(CR_KNOWLEDGE_EXTRACTION_SYSTEM_PACKAGES_FORCE
        "For CMake<3.24: Force using system packages for all dependencies"
        OFF
        "NOT CMAKE_SUPPORTS_TRY_FIND_PACKAGE"
        OFF
)

include(FetchContent)
FetchContent_Declare(
        commonroad_cmake

        GIT_REPOSITORY https://gitlab.lrz.de/tum-cps/commonroad-cmake.git
        GIT_TAG main
)
FetchContent_MakeAvailable(commonroad_cmake)

list(APPEND CMAKE_MODULE_PATH ${commonroad_cmake_SOURCE_DIR})

include(toolchain/DiscoverLLD OPTIONAL)
include(toolchain/DiscoverSanitizers OPTIONAL)

# This is a helper script that will automatically add a .gitignore file to the
# binary directory (build directory) so you don't have to do add every build folder
# to your .gitignore.
include(extras/GitIgnoreBinaryDir OPTIONAL)

if (DEFINED ENV{CIBUILDWHEEL} AND CMAKE_SYSTEM_PROCESSOR MATCHES "i686")
    # Ugly hack for broken pthread detection on manylinux2014_i686
    find_library(OpenMP_pthread_LIBRARY NAMES "pthread")
endif ()

# Required for proper pthread discovery on some systems
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

find_package(Threads REQUIRED)
find_package(OpenMP)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/external)

# We have custom logic to check for system Boost, therefore we include
# ExternalBoost unconditionally
include(external/ExternalBoost)

include(ExternalCLCS)
include(ExternalEnvironmentModel)

if (CMAKE_SUPPORTS_TRY_FIND_PACKAGE)
    if (SKBUILD_SELF_CONTAINED)
        set(FETCHCONTENT_TRY_FIND_PACKAGE_MODE NEVER)
    endif ()
    if (CR_KNOWLEDGE_EXTRACTION_SYSTEM_PACKAGES)
        set(FETCHCONTENT_TRY_FIND_PACKAGE_MODE OPT_IN)
    else ()
        set(FETCHCONTENT_TRY_FIND_PACKAGE_MODE NEVER)
    endif ()
endif ()

# Include eigen3, GTest, and spdlog
if (CR_KNOWLEDGE_EXTRACTION_SYSTEM_PACKAGES_FORCE)
    # This is the fallback branch in case the CMake version is older than 3.24
    # and the user requested we try to use system packages
    # For CMake > 3.24, fallback is automatic through the FetchContent find_package
    # integration.

    message(WARNING "CR_KNOWLEDGE_EXTRACTION_SYSTEM_PACKAGES_FORCE is set - trying to satisfy "
            "all dependencies using installed system packages.\n"
            "If this fails, consider disabling CR_KNOWLEDGE_EXTRACTION_SYSTEM_PACKAGES_FORCE and "
            "trying again.")

    find_package(Eigen3 3.3.7 REQUIRED)
    find_package(spdlog 1.8.0 REQUIRED)
    find_package(GTest)

    # yaml-cpp::yaml-cpp is not present in installed config file as of version 0.7.0
    if (NOT TARGET yaml-cpp::yaml-cpp)
        add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
    endif ()
else ()
    # Normal path: We try to use find_package via FetchContent, otherwise we fall
    # back to normal FetchContent

    include(external/ExternalEigen)
    include(external/ExternalSpdlog)
    include(external/ExternalGoogleTest)
endif ()

# Add subdirectory for the main library
add_subdirectory(cpp)

# Add subdirectory for Python bindings
if (CR_KNOWLEDGE_EXTRACTION_BUILD_PYTHON_BINDINGS)
    add_subdirectory(python_binding)
endif ()

# Add subdirectory for tests
if (CR_KNOWLEDGE_EXTRACTION_BUILD_TESTS)
    enable_testing()
    add_subdirectory(cpp/tests)
endif ()

# include(cmake/install.cmake)
