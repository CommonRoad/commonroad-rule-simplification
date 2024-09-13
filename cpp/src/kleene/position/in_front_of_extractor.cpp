#include "cr_knowledge_extraction/kleene/position/in_front_of_extractor.hpp"

#include <commonroad_cpp/obstacle/obstacle.h>

#include <ranges>

using namespace knowledge_extraction::kleene::position;

std::unordered_map<time_step_t, InFrontOfExtractor::TrueFalseObstacleIds> InFrontOfExtractor::extract(
    const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>> &relevant_obstacle_ids_over_time)
    const {
    std::unordered_map<time_step_t, TrueFalseObstacleIds> true_false_obstacle_ids;
    for (const auto &[time_step, obstacle_ids] : relevant_obstacle_ids_over_time) {
        auto relevant_obstacle_rears =
            world->getObstacles() | std::views::filter([&obstacle_ids](const auto &obstacle) {
                return obstacle_ids.contains(obstacle->getId());
            }) |
            std::views::transform([this, &time_step](const auto &obstacle) {
                return std::make_pair(obstacle->getId(), get_obstacle_rear(time_step, obstacle));
            }) |
            std::views::filter([](const auto &pair) { return pair.second.has_value(); }) |
            std::views::transform([](const auto &pair) { return std::make_pair(pair.first, pair.second.value()); });

        auto ego_front_max = approximations->p_lon_max(time_step) + approximations->get_outer_radius();
        auto ego_front_min = approximations->p_lon_max(time_step) - approximations->get_inner_radius();

        for (const auto &[obstacle_id, rear] : relevant_obstacle_rears) {
            if (ego_front_max < rear) {
                true_false_obstacle_ids[time_step].first.emplace(obstacle_id);
            } else if (ego_front_min >= rear) {
                true_false_obstacle_ids[time_step].second.emplace(obstacle_id);
            }
        }
    }
    return true_false_obstacle_ids;
}

std::optional<double> InFrontOfExtractor::get_obstacle_rear(size_t time_step,
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
