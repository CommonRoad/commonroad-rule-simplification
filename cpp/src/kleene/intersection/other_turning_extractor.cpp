#include "cr_knowledge_extraction/kleene/intersection/other_turning_extractor.hpp"

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>

#include <ranges>

using namespace knowledge_extraction::kleene::intersection;

std::unordered_map<time_step_t, OtherTurningExtractor::TrueFalseObstacleIds> OtherTurningExtractor::extract(
    const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>> &relevant_obstacle_ids_over_time)
    const {
    const auto &road_network = env_model->get_world()->getRoadNetwork();

    // All optionals should have values, since this extractor is not triggered for the ego vehicle
    auto relevant_obstacle_ids_ = relevant_obstacle_ids_over_time | std::views::values | std::views::join |
                                  std::views::transform([](const auto &opt) { return opt.value(); });
    std::unordered_set<size_t> relevant_obstacle_ids{relevant_obstacle_ids_.begin(), relevant_obstacle_ids_.end()};
    auto relevant_obstacles =
        env_model->get_world()->getObstacles() | std::views::filter([&relevant_obstacle_ids](const auto &obs) {
            return relevant_obstacle_ids.contains(obs->getId());
        });

    TrueFalseObstacleIds true_false_obstacle_ids;
    for (const auto &obstacle : relevant_obstacles) {
        if (env_model->get_turning_directions(obstacle).contains(direction)) {
            true_false_obstacle_ids.first.insert(obstacle->getId());
        } else {
            true_false_obstacle_ids.second.insert(obstacle->getId());
        }
    }

    std::unordered_map<time_step_t, TrueFalseObstacleIds> result;
    result.reserve(relevant_obstacle_ids_over_time.size());
    for (const auto &time_step : relevant_obstacle_ids_over_time | std::views::keys) {
        result.emplace(time_step, true_false_obstacle_ids);
    }
    return result;
}
