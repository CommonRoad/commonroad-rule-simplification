include(FetchContent)

# Force the environment model to always use the external drivability checker
set(EXTERNAL_CRDC_FORCE ON)

FetchContent_Declare(
        EnvironmentModel
        GIT_REPOSITORY git@gitlab.lrz.de:cps/commonroad/environment-model.git
        GIT_TAG ef1efc16f50c3f49a12456a516e5914617bf93e1
        #    URL /home/lercher/tum/commonroad/environment-model
)

FetchContent_MakeAvailable(EnvironmentModel)

# Only build targets of environment model if they are required by one of our targets
set_property(DIRECTORY ${EnvironmentModel_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)
