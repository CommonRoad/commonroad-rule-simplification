include(FetchContent)

FetchContent_Declare(
        EnvironmentModel
        SYSTEM

        GIT_REPOSITORY git@gitlab.lrz.de:cps/commonroad/environment-model.git
        GIT_TAG c7fcf8ad0855da4958df3045b3f59a6e482bab6e  # if you change the commit here also update the Python dependency
)

FetchContent_MakeAvailable(EnvironmentModel)

# Only build targets of environment model if they are required by one of our targets
set_property(DIRECTORY ${EnvironmentModel_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)
