#include "cr_knowledge_extraction/kleene/braking/safe_distance_extractor.hpp"

#include <commonroad_cpp/obstacle/obstacle.h>

#include <ranges>

using namespace knowledge_extraction::kleene::braking;

std::unordered_map<time_step_t, SafeDistanceExtractor::TrueFalseObstacleIds> SafeDistanceExtractor::extract(
    const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>> &relevant_obstacle_ids_over_time)
    const {
    std::unordered_map<time_step_t, TrueFalseObstacleIds> true_false_obstacle_ids;
    auto a_min_ego = env_model->get_ego_params().a_lon_min;
    for (const auto &[time_step, obstacle_ids] : relevant_obstacle_ids_over_time) {
        auto relevant_obstacle_stopping_s =
            env_model->get_world()->getObstacles() | std::views::filter([&obstacle_ids](const auto &obstacle) {
                return obstacle_ids.contains(obstacle->getId());
            }) |
            std::views::transform([this, &time_step, &a_min_ego](const auto &obstacle) {
                assert(obstacle->getAminLong() < a_min_ego);
                return std::make_pair(obstacle->getId(), env_model->get_stopping_s(time_step, obstacle));
            }) |
            std::views::filter([](const auto &pair) { return pair.second.has_value(); }) |
            std::views::transform([](const auto &pair) { return std::make_pair(pair.first, pair.second.value()); });

        const auto &approximations = env_model->get_ego_approximations();
        auto ego_stopping_s_max = approximations->p_lon_max(time_step) +
                                  compute_ego_stopping_distance(approximations->v_max(time_step)) +
                                  approximations->get_outer_radius();
        auto ego_stopping_s_min = approximations->p_lon_min(time_step) +
                                  compute_ego_stopping_distance(approximations->v_min(time_step)) +
                                  approximations->get_inner_radius();

        for (const auto &[obstacle_id, stopping_s] : relevant_obstacle_stopping_s) {
            if (ego_stopping_s_max < stopping_s) {
                true_false_obstacle_ids[time_step].first.emplace(obstacle_id);
            } else if (ego_stopping_s_min >= stopping_s) {
                true_false_obstacle_ids[time_step].second.emplace(obstacle_id);
            }
        }
    }
    return true_false_obstacle_ids;
}

double SafeDistanceExtractor::compute_ego_stopping_distance(double initial_v) const {
    auto clamped_v = std::max(initial_v, 0.0);
    auto braking_distance = (clamped_v * clamped_v) / (2 * std::abs(env_model->get_ego_params().a_lon_min));
    auto reaction_distance = env_model->get_ego_params().t_react * clamped_v;
    return braking_distance + reaction_distance;
}
