#include "cr_knowledge_extraction/kleene/ego_independent/ego_independent_extractor.hpp"

using namespace knowledge_extraction::kleene::ego_independent;

std::unordered_map<time_step_t, EgoIndependentExtractor::TrueFalseObstacleIds> EgoIndependentExtractor::extract(
    const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>> &relevant_obstacle_ids_over_time)
    const {
    std::unordered_map<time_step_t, TrueFalseObstacleIds> true_false_obstacle_ids;
    for (const auto &[time_step, obstacle_ids] : relevant_obstacle_ids_over_time) {
        for (const auto &obstacle_id : obstacle_ids) {
            if (evaluate_inner(time_step, obstacle_id.value())) {
                true_false_obstacle_ids[time_step].first.insert(obstacle_id);
            } else {
                true_false_obstacle_ids[time_step].second.insert(obstacle_id);
            }
        }
    }
    return true_false_obstacle_ids;
}

bool EgoIndependentExtractor::evaluate_inner(time_step_t step, size_t obstacle_id) const {
    auto obstacle = world->findObstacle(obstacle_id);
    return inner_predicate->booleanEvaluation(step, world, obstacle, nullptr, additional_params);
}
