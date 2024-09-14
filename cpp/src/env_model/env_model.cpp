#include "cr_knowledge_extraction/env_model/env_model.hpp"

#include <commonroad_cpp/geometry/geometric_operations.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>

using namespace knowledge_extraction::env_model;

std::shared_ptr<knowledge_extraction::ego_behavior::BehaviorOverapproximation>
EnvironmentModel::make_ego_approximations(const std::shared_ptr<World> &world,
                                          const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs,
                                          knowledge_extraction::ego_behavior::EgoParameters ego_params) {
    auto &initial_state = ego_params.initial_state;

    auto ccs_pos = ego_ccs->convertToCurvilinearCoords(initial_state.getXPosition(), initial_state.getYPosition());
    initial_state.setLonPosition(ccs_pos.x());
    initial_state.setLatPosition(ccs_pos.y());

    auto ccs_tangent = ego_ccs->tangent(ccs_pos.x());
    auto ccs_orientation = std::atan2(ccs_tangent.y(), ccs_tangent.x());
    auto theta = geometric_operations::subtractOrientations(initial_state.getGlobalOrientation(), ccs_orientation);
    initial_state.setCurvilinearOrientation(theta);

    auto dt = world->getDt();

    return std::make_shared<ego_behavior::BehaviorOverapproximation>(
        dt, ego_params, road_network::CurvilinearRoadNetwork{world, ego_ccs});
}

std::optional<double> EnvironmentModel::get_obstacle_rear_impl(size_t time_step,
                                                               const std::shared_ptr<Obstacle> &obstacle) const {
    std::shared_ptr<State> obstacle_state;
    try {
        obstacle_state = obstacle->getStateByTimeStep(time_step);
    } catch (std::logic_error &e) {
        return std::nullopt;
    }
    if (!ego_ccs->cartesianPointInProjectionDomain(obstacle_state->getXPosition(), obstacle_state->getYPosition())) {
        return std::nullopt;
    }

    return obstacle->rearS(time_step, ego_ccs);
}

std::optional<double> EnvironmentModel::get_obstacle_rear(size_t time_step, const std::shared_ptr<Obstacle> &obstacle) {
    auto obstacle_id = obstacle->getId();
    auto key = std::make_pair(time_step, obstacle_id);
    if (obstacle_rear_cache.contains(key)) {
        return obstacle_rear_cache.at(key);
    }

    auto result = get_obstacle_rear_impl(time_step, obstacle);

    obstacle_rear_cache.emplace(key, result);

    return result;
}

std::optional<std::set<size_t>>
EnvironmentModel::get_obstacle_lane_ids_impl(size_t time_step, const std::shared_ptr<Obstacle> &obstacle) const {
    std::shared_ptr<State> obstacle_state;
    try {
        std::set<size_t> lanelet_ids{};
        auto occupied_lanes = obstacle->getOccupiedLanes(world->getRoadNetwork(), time_step);
        for (const auto &lane : occupied_lanes) {
            auto lane_lanelet_ids = lane->getContainedLaneletIDs();
            lanelet_ids.insert(lane_lanelet_ids.begin(), lane_lanelet_ids.end());
        }
        return lanelet_ids;
    } catch (std::logic_error &e) {
        return std::nullopt;
    }
}

std::optional<std::set<size_t>> EnvironmentModel::get_obstacle_lane_ids(size_t time_step,
                                                                        const std::shared_ptr<Obstacle> &obstacle) {
    auto obstacle_id = obstacle->getId();
    auto key = std::make_pair(time_step, obstacle_id);
    if (obstacle_lane_ids_cache.contains(key)) {
        return obstacle_lane_ids_cache.at(key);
    }

    auto result = get_obstacle_lane_ids_impl(time_step, obstacle);

    obstacle_lane_ids_cache.emplace(key, result);

    return result;
}
