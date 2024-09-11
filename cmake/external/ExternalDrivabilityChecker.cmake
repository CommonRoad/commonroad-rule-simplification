include(FetchContent)

find_package(Threads REQUIRED)

# Disable pybind dependency of drivability checker
set(ADD_PYTHON_BINDINGS OFF)
set(BUILD_PYBIND11 OFF)

FetchContent_Declare(
    crdc
    GIT_REPOSITORY  https://github.com/CommonRoad/commonroad-drivability-checker.git
    GIT_TAG         14dab05029aa60ada83b3d7ea56f2a8b81c989e1
    #GIT_TAG        development
)

FetchContent_MakeAvailable(crdc)

set_property(DIRECTORY ${crdc_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)

mark_as_advanced(
    ADD_MODULE_GEOMETRY
    ADD_MODULE_COLLISION
    ADD_TRIANGLE
    BUILD_S11N
)
