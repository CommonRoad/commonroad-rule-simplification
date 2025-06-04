include(FetchContent)

# we explicitly set build s11n to ON since it is deactivated by default in the CMake file of the CLCS if it is not top level
set(CR_CLCS_BUILD_S11N ON)

FetchContent_Declare(
        CommonRoadCLCS
        SYSTEM
        # Release tag v2025.2.0
        GIT_REPOSITORY https://github.com/CommonRoad/commonroad-clcs.git
        GIT_TAG 34040496aa71a6244c99ce3236d3c395731b940b
)

FetchContent_MakeAvailable(CommonRoadCLCS)

set_property(DIRECTORY ${CommonRoadCLCS_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)

mark_as_advanced(
        CR_CLCS_BUILD_S11N
)
