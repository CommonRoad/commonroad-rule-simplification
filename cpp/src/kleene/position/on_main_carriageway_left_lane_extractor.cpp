#include "cr_knowledge_extraction/kleene/position/on_main_carriageway_left_lane_extractor.hpp"

#include <ranges>

using namespace knowledge_extraction::kleene::position;

std::unordered_map<time_step_t, OnMainCarriagewayLeftLaneExtractor::TrueFalseObstacleIds>
OnMainCarriagewayLeftLaneExtractor::extract(
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
                return is_mcw(lanelet) && (is_leftmost(lanelet) || is_neighbour_opposite(lanelet));
            });
        if (must_be_true) {
            true_false_obstacle_ids[time_step].first.insert(std::nullopt);
            continue;
        }
    }
    return true_false_obstacle_ids;
}

bool OnMainCarriagewayLeftLaneExtractor::is_mcw(const std::shared_ptr<Lanelet> &lanelet) {
    return lanelet->hasLaneletType(LaneletType::mainCarriageWay);
}

bool OnMainCarriagewayLeftLaneExtractor::is_leftmost(const std::shared_ptr<Lanelet> &lanelet) {
    return lanelet->getAdjacentLeft().adj == nullptr ||
           lanelet->getAdjacentLeft().oppositeDir != lanelet->getAdjacentLeft().adj->getAdjacentRight().oppositeDir;
}

bool OnMainCarriagewayLeftLaneExtractor::is_neighbour_opposite(const std::shared_ptr<Lanelet> &lanelet) {
    return lanelet->getAdjacentLeft().oppositeDir;
}
