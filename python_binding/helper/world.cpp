#include "world.hpp"

#include <commonroad_cpp/roadNetwork/road_network.h>

std::shared_ptr<World> pybind_helper::open_world(const std::string &scenario_path) {
    // Create environment model
    const auto &[obstacles, roadNetwork, step_size] = InputUtils::getDataFromCommonRoad(scenario_path);
    // World always starts at time 0, and we don't have any ego vehicles in the world
    return std::make_shared<World>("knowledgeExtractionWorld", 0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{},
                                   obstacles, step_size);
}
