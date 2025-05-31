include(FetchContent)

FetchContent_Declare(
        CommonRoadCLCS
        SYSTEM

        GIT_REPOSITORY https://github.com/CommonRoad/commonroad-clcs.git
        GIT_TAG 3bbfc0de99778b17e725ce2f259b989e751b937b
)

FetchContent_MakeAvailable(CommonRoadCLCS)

set_property(DIRECTORY ${CommonRoadCLCS_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)
