#include "cr_knowledge_extraction/kleene/position/on_main_carriageway_right_lane_extractor.hpp"

#include <ranges>

using namespace knowledge_extraction::kleene::position;

std::unordered_map<time_step_t, OnMainCarriagewayRightLaneExtractor::TrueFalseObstacleIds>
OnMainCarriagewayRightLaneExtractor::extract(
    const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>> &relevant_obstacle_ids_over_time)
    const {
    std::unordered_map<time_step_t, TrueFalseObstacleIds> true_false_obstacle_ids;
    for (const auto &[time_step, obstacle_ids] : relevant_obstacle_ids_over_time) {
        // Should only contain std::nullopt as this predicate does not have parameters
        assert(obstacle_ids.size() == 1);

        const auto &approximations = env_model->get_ego_approximations();

        auto cannot_be_true = std::ranges::all_of(approximations->get_covered_lanelets(time_step),
                                                  [](const auto &lanelet) { return !is_mcw(lanelet); });
        if (cannot_be_true) {
            true_false_obstacle_ids[time_step].second.insert(std::nullopt);
            continue;
        }

        auto must_be_true =
            std::ranges::all_of(approximations->get_intersected_lanelets(time_step), [](const auto &lanelet) {
                return is_mcw(lanelet) && (is_rightmost(lanelet) || is_neighbour_opposite(lanelet));
            });
        if (must_be_true) {
            true_false_obstacle_ids[time_step].first.insert(std::nullopt);
            continue;
        }
    }
    return true_false_obstacle_ids;
}

bool OnMainCarriagewayRightLaneExtractor::is_mcw(const std::shared_ptr<Lanelet> &lanelet) {
    return lanelet->hasLaneletType(LaneletType::mainCarriageWay);
}

bool OnMainCarriagewayRightLaneExtractor::is_rightmost(const std::shared_ptr<Lanelet> &lanelet) {
    return lanelet->getAdjacentRight().adj == nullptr ||
           lanelet->getAdjacentRight().oppositeDir != lanelet->getAdjacentRight().adj->getAdjacentLeft().oppositeDir;
}

bool OnMainCarriagewayRightLaneExtractor::is_neighbour_opposite(const std::shared_ptr<Lanelet> &lanelet) {
    return lanelet->getAdjacentRight().oppositeDir;
}
