#include "cr_knowledge_extraction/kleene/position/in_same_lane_extractor.hpp"

#include <commonroad_cpp/obstacle/obstacle.h>

#include <ranges>

using namespace knowledge_extraction::kleene::position;

std::unordered_map<time_step_t, InSameLaneExtractor::TrueFalseObstacleIds> InSameLaneExtractor::extract(
    const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>> &relevant_obstacle_ids_over_time)
    const {
    std::unordered_map<time_step_t, TrueFalseObstacleIds> true_false_obstacle_ids;
    for (const auto &[time_step, obstacle_ids] : relevant_obstacle_ids_over_time) {
        auto relevant_obstacle_lanes =
            env_model->get_world()->getObstacles() | std::views::filter([&obstacle_ids](const auto &obstacle) {
                return obstacle_ids.contains(obstacle->getId());
            }) |
            std::views::transform([this, &time_step](const auto &obstacle) {
                return std::make_pair(obstacle->getId(), env_model->get_obstacle_lane_ids(time_step, obstacle));
            }) |
            std::views::filter([](const auto &pair) { return pair.second.has_value(); }) |
            std::views::transform([](const auto &pair) { return std::make_pair(pair.first, pair.second.value()); });

        const auto &approximations = env_model->get_ego_approximations();
        const auto &ego_covered_lanelets = approximations->get_covered_lanelets(time_step);
        const auto &ego_intersected_lanelets = approximations->get_intersected_lanelets(time_step);

        for (const auto &[obstacle_id, lanelet_ids] : relevant_obstacle_lanes) {
            auto cannot_be_true = std::ranges::none_of(ego_covered_lanelets, [&lanelet_ids](const auto &lanelet) {
                return lanelet_ids.contains(lanelet->lanelet->getId());
            });
            if (cannot_be_true) {
                true_false_obstacle_ids[time_step].second.emplace(obstacle_id);
                continue;
            }

            auto must_be_true = std::ranges::all_of(ego_intersected_lanelets, [&lanelet_ids](const auto &lanelet) {
                return lanelet_ids.contains(lanelet->lanelet->getId());
            });
            if (must_be_true) {
                true_false_obstacle_ids[time_step].first.emplace(obstacle_id);
                continue;
            }
        }
    }
    return true_false_obstacle_ids;
}
