#include "cr_knowledge_extraction/kleene/ego_independent/ego_independent_extractor.hpp"

#include <commonroad_cpp/obstacle/obstacle.h>
#include <spdlog/spdlog.h>

using namespace knowledge_extraction::kleene::ego_independent;

std::unordered_map<time_step_t, EgoIndependentExtractor::TrueFalseObstacleIds> EgoIndependentExtractor::extract(
    const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>> &relevant_obstacle_ids_over_time)
    const {
    std::unordered_map<time_step_t, TrueFalseObstacleIds> true_false_obstacle_ids;
    for (const auto &[time_step, obstacle_ids] : relevant_obstacle_ids_over_time) {
        for (const auto &obstacle : env_model->get_world()->getObstacles()) {
            const auto obstacle_id = obstacle->getId();
            if (!obstacle_ids.contains(obstacle_id)) {
                continue;
            }
            auto inner_result = evaluate_inner(time_step, obstacle);
            if (inner_result.has_value()) {
                if (inner_result.value()) {
                    true_false_obstacle_ids[time_step].first.insert(obstacle_id);
                } else {
                    true_false_obstacle_ids[time_step].second.insert(obstacle_id);
                }
            }
        }
    }
    return true_false_obstacle_ids;
}

std::optional<bool> EgoIndependentExtractor::evaluate_inner(time_step_t step,
                                                            const std::shared_ptr<Obstacle> &obstacle) const {
    try {
        return inner_predicate->booleanEvaluation(step, env_model->get_world(), obstacle, nullptr, additional_params);
    } catch (std::exception &e) {
        spdlog::warn("Evaluation of CommonRoad predicate failed: {}", e.what());
        return std::nullopt;
    }
}
