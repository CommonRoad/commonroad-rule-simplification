#pragma once

#include <commonroad_cpp/world.h>
#include <geometry/curvilinear_coordinate_system.h>

struct TestEnvironments {
    using Environment = std::pair<std::shared_ptr<World>, std::shared_ptr<geometry::CurvilinearCoordinateSystem>>;

    Environment interstate_simple;

    TestEnvironments();

    static const std::string test_scenario_dir;
    static Environment setup_interstate_simple();
};
