#include "cr_knowledge_extraction/kleene/position/at_traffic_sign_extractor.hpp"

#include <commonroad_cpp/roadNetwork/road_network.h>

#include <ranges>

using namespace knowledge_extraction::kleene::position;

std::unordered_map<time_step_t, AtTrafficSignExtractor::TrueFalseObstacleIds> AtTrafficSignExtractor::extract(
    const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>> &relevant_obstacle_ids_over_time)
    const {

    auto relevant_lanelet_ids_ = env_model->get_world()->getRoadNetwork()->getLaneletNetwork() |
                                 std::views::filter([this](const auto &lanelet) {
                                     return std::ranges::any_of(lanelet->getTrafficSigns(), [this](const auto &sign) {
                                         return !sign->getTrafficSignElementsOfType(traffic_sign_type).empty();
                                     });
                                 }) |
                                 std::views::transform([](const auto &lanelet) { return lanelet->getId(); });
    std::unordered_set<size_t> relevant_lanelet_ids{relevant_lanelet_ids_.begin(), relevant_lanelet_ids_.end()};

    std::unordered_map<time_step_t, TrueFalseObstacleIds> true_false_obstacle_ids;
    for (const auto &[time_step, obstacle_ids] : relevant_obstacle_ids_over_time) {
        // Should only contain std::nullopt as this predicate does not have parameters
        assert(obstacle_ids.size() == 1);

        if (relevant_lanelet_ids.empty()) {
            true_false_obstacle_ids[time_step].second.insert(std::nullopt);
            continue;
        }

        const auto &approximations = env_model->get_ego_approximations();

        auto cannot_be_true = std::ranges::none_of(
            approximations->get_covered_lanelets(time_step), [&relevant_lanelet_ids](const auto &ccs_lanelet) {
                return relevant_lanelet_ids.contains(ccs_lanelet->lanelet->getId());
            });
        if (cannot_be_true) {
            true_false_obstacle_ids[time_step].second.insert(std::nullopt);
            continue;
        }

        auto must_be_true = std::ranges::all_of(approximations->get_intersected_lanelets(time_step),
                                                [&relevant_lanelet_ids](const auto &ccs_lanelet) {
                                                    return relevant_lanelet_ids.contains(ccs_lanelet->lanelet->getId());
                                                });
        if (must_be_true) {
            true_false_obstacle_ids[time_step].first.insert(std::nullopt);
            continue;
        }
    }
    return true_false_obstacle_ids;
}
