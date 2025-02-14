#include "cr_knowledge_extraction/env_model/env_model.hpp"

#include <commonroad_cpp/geometry/geometric_operations.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/lane/on_similar_oriented_lanelet_with_type_predicate.h>
#include <commonroad_cpp/predicates/lane/on_similar_oriented_lanelet_without_type_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>

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

    return std::make_shared<ego_behavior::BehaviorOverapproximation>(
        world->getDt(), ego_params, road_network::CurvilinearRoadNetwork{world->getRoadNetwork(), ego_ccs});
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
        auto occupied_lanes = obstacle->getOccupiedLanesDrivingDirection(world->getRoadNetwork(), time_step);
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

std::optional<double> EnvironmentModel::get_stopping_s_impl(size_t time_step,
                                                            const std::shared_ptr<Obstacle> &obstacle) {
    auto rear_opt = get_obstacle_rear(time_step, obstacle);
    if (!rear_opt.has_value()) {
        return std::nullopt;
    }
    auto rear = rear_opt.value();

    // This cannot fail, otherwise get_obstacle_rear would have returned std::nullopt
    auto velocity = obstacle->getStateByTimeStep(time_step)->getVelocity();

    return rear + ((velocity * velocity) / (2 * std::abs(obstacle->getAminLong())));
}

std::optional<double> EnvironmentModel::get_stopping_s(size_t time_step, const std::shared_ptr<Obstacle> &obstacle) {
    auto obstacle_id = obstacle->getId();
    auto key = std::make_pair(time_step, obstacle_id);
    if (stopping_s_cache.contains(key)) {
        return stopping_s_cache.at(key);
    }

    auto result = get_stopping_s_impl(time_step, obstacle);

    stopping_s_cache.emplace(key, result);

    return result;
}

std::unordered_set<Direction> EnvironmentModel::get_turning_directions_impl(const std::shared_ptr<Obstacle> &obstacle) {
    auto on_lanelet_with_type = OnSimilarOrientedLaneletWithTypePredicate{};
    auto not_on_lanelet_with_type = OnSimilarOrientedLaneletWithoutTypePredicate{};

    // State tracks:
    // - Mode: 0 = Before incoming, 1 = On incoming, 2 = In intersection, 3 = Done
    // - left, straight, right: Whether the respective direction is still possible
    std::tuple<int, bool, bool, bool> state = {0, true, true, true};

    // An obstacle turns (left/straight/right) if it leaves an incoming at some point and then
    // stays on (left/straight/right) lanelets until it leaves the intersection (the until here is weak)
    // cf. meta predicates in crmonitor
    for (const auto &time_step : obstacle->getTimeSteps()) {
        auto &[mode, left, straight, right] = state;
        if (mode == 3) {
            break;
        }
        auto on_incoming = [&]() {
            return on_lanelet_with_type.booleanEvaluation(time_step, world, obstacle, {}, {"incoming"});
        };
        auto on_dir_lanelet = [&](std::string dir) {
            return on_lanelet_with_type.booleanEvaluation(time_step, world, obstacle, {}, {std::move(dir)});
        };
        auto not_on_intersection = [&]() {
            return not_on_lanelet_with_type.booleanEvaluation(time_step, world, obstacle, {}, {"intersection"});
        };
        switch (mode) {
        case 0:
            // Before incoming
            if (on_incoming()) {
                mode = 1;
            }
            break;
        case 1:
            // On incoming
            if (on_incoming()) {
                // We stay before the intersection
                break;
            }
            // We have left the incoming and entered the intersection
            mode = 2;
            // Intentional fall-through to the next case
            // we have to check on what kind of direction lanelet the obstacle is at this step
        case 2:
            // In intersection
            if (not_on_intersection()) {
                // We are done
                mode = 3;
            } else {
                // If we leave a lanelet for a direction, that direction is no longer possible
                if (right && !on_dir_lanelet("right")) {
                    right = false;
                }
                if (straight && !on_dir_lanelet("straight")) {
                    straight = false;
                }
                if (left && !on_dir_lanelet("left")) {
                    left = false;
                }
                if (!left && !straight && !right) {
                    // We are done
                    mode = 3;
                }
            }
            break;
        }
    }

    // We are done, directions that do not yet have a value are set to true, since we have a weak until
    const auto &[mode, left, straight, right] = state;
    std::unordered_set<Direction> directions;
    if (mode >= 2) {
        // In mode 0 or 1, we have not entered the intersection yet, so we did not fulfill the eventuality
        if (left) {
            directions.insert(Direction::left);
        }
        if (straight) {
            directions.insert(Direction::straight);
        }
        if (right) {
            directions.insert(Direction::right);
        }
    }

    return directions;
}

const std::unordered_set<Direction> &
EnvironmentModel::get_turning_directions(const std::shared_ptr<Obstacle> &obstacle) {
    auto obstacle_id = obstacle->getId();
    if (turning_directions_cache.contains(obstacle_id)) {
        return turning_directions_cache.at(obstacle_id);
    }

    auto result = get_turning_directions_impl(obstacle);

    turning_directions_cache.emplace(obstacle_id, result);

    return turning_directions_cache.at(obstacle_id);
}

std::optional<int> EnvironmentModel::get_priority(size_t time_step, const std::shared_ptr<Obstacle> &obstacle,
                                                  Direction dir) {
    auto obstacle_id = obstacle->getId();
    auto key = std::make_tuple(time_step, obstacle_id, dir);
    if (priority_cache.contains(key)) {
        return priority_cache.at(key);
    }

    auto priority = regulatory_elements_utils::getPriority(time_step, world->getRoadNetwork(), obstacle, dir);

    priority_cache.emplace(key, priority);

    return priority;
}
