#include "cr_knowledge_extraction/kleene/position/relevant_traffic_light_extractor.hpp"

#include <commonroad_cpp/roadNetwork/road_network.h>

#include <ranges>

using namespace knowledge_extraction::kleene::position;

std::unordered_map<time_step_t, RelevantTrafficLightExtractor::TrueFalseObstacleIds>
RelevantTrafficLightExtractor::extract(const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>>
                                           &relevant_obstacle_ids_over_time) const {

    bool scenario_has_traffic_lights = !env_model->get_world()->getRoadNetwork()->getTrafficLights().empty();

    std::unordered_map<time_step_t, TrueFalseObstacleIds> true_false_obstacle_ids;
    for (const auto &[time_step, obstacle_ids] : relevant_obstacle_ids_over_time) {
        // Should only contain std::nullopt as this predicate does not have parameters
        assert(obstacle_ids.size() == 1);

        if (!scenario_has_traffic_lights) {
            true_false_obstacle_ids[time_step].second.emplace(std::nullopt);
            continue;
        }

        // TODO: More sophisticated extraction
    }
    return true_false_obstacle_ids;
}
