include(FetchContent)

# Force the environment model to always use the external drivability checker
set(EXTERNAL_CRDC_FORCE ON)

FetchContent_Declare(
        EnvironmentModel
        GIT_REPOSITORY git@gitlab.lrz.de:cps/commonroad/environment-model.git
        GIT_TAG 81a1da3442798b658c654c7e54dcbea21111aee2
        #    URL /home/lercher/tum/commonroad/environment-model
)

FetchContent_MakeAvailable(EnvironmentModel)

# Only build targets of environment model if they are required by one of our targets
set_property(DIRECTORY ${EnvironmentModel_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)
