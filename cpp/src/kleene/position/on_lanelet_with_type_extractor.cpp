#include "cr_knowledge_extraction/kleene/position/on_lanelet_with_type_extractor.hpp"
#include "cr_knowledge_extraction/kleene/ego_dependent_kleene_extractor.hpp"

#include <ranges>

using namespace knowledge_extraction::kleene::position;

std::unordered_map<time_step_t, OnLaneletWithTypeExtractor::TrueFalseObstacleIds> OnLaneletWithTypeExtractor::extract(
    const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>> &relevant_obstacle_ids_over_time)
    const {
    std::unordered_map<time_step_t, TrueFalseObstacleIds> true_false_obstacle_ids;
    for (const auto &[time_step, obstacle_ids] : relevant_obstacle_ids_over_time) {
        // Should only contain std::nullopt as this predicate does not have parameters
        assert(obstacle_ids.size() == 1);

        auto cannot_be_true =
            std::ranges::none_of(approximations->get_covered_lanelets(time_step), [this](const auto &ccs_lanelet) {
                return ccs_lanelet->lanelet->getLaneletTypes().contains(lanelet_type);
            });
        if (cannot_be_true) {
            true_false_obstacle_ids[time_step].second.insert(std::nullopt);
            continue;
        }

        auto must_be_true =
            std::ranges::all_of(approximations->get_intersected_lanelets(time_step), [this](const auto &ccs_lanelet) {
                return ccs_lanelet->lanelet->getLaneletTypes().contains(lanelet_type);
            });
        if (must_be_true) {
            true_false_obstacle_ids[time_step].first.insert(std::nullopt);
            continue;
        }
    }
    return true_false_obstacle_ids;
}
