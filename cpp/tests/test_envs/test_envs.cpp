#include "test_envs.hpp"

#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

using knowledge_extraction::ego_behavior::EgoParameters;
using knowledge_extraction::env_model::EnvironmentModel;

TestEnvironments::TestEnvironments() : interstate_simple(setup_interstate_simple()), two_lanes(setup_two_lanes()) {}

const std::string TestEnvironments::test_scenario_dir = "../cpp/tests/scenarios/";

std::shared_ptr<EnvironmentModel> TestEnvironments::setup_interstate_simple() {
    auto scenario = InputUtils::getDataFromCommonRoad(test_scenario_dir + "interstate_simple.xml");
    auto world =
        std::make_shared<World>("interstate_simple", 0, scenario.roadNetwork, std::vector<std::shared_ptr<Obstacle>>{},
                                scenario.obstacles, scenario.timeStepSize);

    // reference path aligned with cartesian axes so that CCS coordinates are equal to cartesian coordinates
    geometry::EigenPolyline reference_path{{-250, 0}, {0, 0}, {250, 0}};
    auto ccs = std::make_shared<geometry::CurvilinearCoordinateSystem>(reference_path, 100);

    return std::make_shared<EnvironmentModel>(world, ccs, EgoParameters{}, PredicateParameters{});
}

std::shared_ptr<EnvironmentModel> TestEnvironments::setup_two_lanes() {
    auto scenario = InputUtils::getDataFromCommonRoad(test_scenario_dir + "two_lanes.xml");
    auto world = std::make_shared<World>("two_lanes", 0, scenario.roadNetwork, std::vector<std::shared_ptr<Obstacle>>{},
                                         scenario.obstacles, scenario.timeStepSize);

    // reference path aligned with cartesian axes so that CCS coordinates are equal to cartesian coordinates
    geometry::EigenPolyline reference_path{{-250, 0}, {0, 0}, {250, 0}};
    auto ccs = std::make_shared<geometry::CurvilinearCoordinateSystem>(reference_path, 100);

    return std::make_shared<EnvironmentModel>(world, ccs, EgoParameters{}, PredicateParameters{});
}
