#pragma once

#include "cr_knowledge_extraction/env_model/env_model.hpp"

#include <commonroad_cpp/world.h>
#include <geometry/curvilinear_coordinate_system.h>

struct TestEnvironments {
    std::shared_ptr<knowledge_extraction::env_model::EnvironmentModel> interstate_simple;

    TestEnvironments();

    static const std::string test_scenario_dir;
    static std::shared_ptr<knowledge_extraction::env_model::EnvironmentModel> setup_interstate_simple();
};
