#include "cr_knowledge_extraction/kleene/general/cut_in_extractor.hpp"

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/lane/in_single_lane_predicate.h>

#include <ranges>

using namespace knowledge_extraction::kleene::general;

std::unordered_map<time_step_t, CutInExtractor::TrueFalseObstacleIds> CutInExtractor::extract(
    const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>> &relevant_obstacle_ids_over_time)
    const {
    std::unordered_map<time_step_t, TrueFalseObstacleIds> true_false_obstacle_ids;
    auto in_single_lane = InSingleLanePredicate{};
    for (const auto &[time_step, obstacle_ids] : relevant_obstacle_ids_over_time) {
        for (const auto &obstacle : env_model->get_world()->getObstacles()) {
            if (!obstacle_ids.contains(obstacle->getId())) {
                continue;
            }

            // Is obstacle in more than one lane?
            bool is_in_single_lane;
            try {
                is_in_single_lane = in_single_lane.booleanEvaluation(time_step, env_model->get_world(), obstacle);
            } catch (const std::logic_error &e) {
                // If the time step does not exist, we don't extract any knowledge
                continue;
            }
            if (is_in_single_lane) {
                // There cannot be a cut in, if the obstacle only occupies a single lane
                true_false_obstacle_ids[time_step].second.emplace(obstacle->getId());
                continue;
            }

            // Is obstacle in the same lane as the ego?
            // This cannot be std::nullopt, as the occupied lanes did not throw an exception above
            auto obstacle_lanelet_ids = env_model->get_obstacle_lane_ids(time_step, obstacle).value();
            const auto &ego_covered_lanelets = env_model->get_ego_approximations()->get_covered_lanelets(time_step);
            auto cannot_be_true =
                std::ranges::none_of(ego_covered_lanelets, [&obstacle_lanelet_ids](const auto &lanelet) {
                    return obstacle_lanelet_ids.contains(lanelet->getId());
                });
            if (cannot_be_true) {
                true_false_obstacle_ids[time_step].second.emplace(obstacle->getId());
                continue;
            }

            // We could do further checks here, but they are too expensive for the knowledge they provide
            // Thus, we just extract no knowledge in this case
        }
    }
    return true_false_obstacle_ids;
}
