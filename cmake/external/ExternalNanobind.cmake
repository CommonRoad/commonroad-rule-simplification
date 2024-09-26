include_guard(DIRECTORY)

include(FetchContent)
include(utils/FetchContentHelper)

FetchContent_Declare_Fallback(
        nanobind

        GIT_REPOSITORY https://github.com/wjakob/nanobind.git
        GIT_TAG v2.1.0

        FIND_PACKAGE_ARGS
)

FetchContent_MakeAvailable(nanobind)
