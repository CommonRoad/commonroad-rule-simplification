include(FetchContent)

FetchContent_Declare(
        EnvironmentModel
        SYSTEM
        GIT_REPOSITORY https://github.com/CommonRoad/environment-model.git
        GIT_TAG develop  # if you change the commit here also update the Python dependency
)

FetchContent_MakeAvailable(EnvironmentModel)

# Only build targets of environment model if they are required by one of our targets
set_property(DIRECTORY ${EnvironmentModel_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)
