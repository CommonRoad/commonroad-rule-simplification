#include "cr_knowledge_extraction/kleene/regulatory/priority_extractor.hpp"

#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/intersection/intersection.h"
#include "commonroad_cpp/roadNetwork/lanelet/lane.h"

#include <ranges>

using namespace knowledge_extraction::kleene::regulatory;

std::unordered_map<time_step_t, PriorityExtractor::TrueFalseObstacleIds> PriorityExtractor::extract(
    const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>> &relevant_obstacle_ids_over_time)
    const {

    std::unordered_map<time_step_t, TrueFalseObstacleIds> true_false_obstacle_ids;
    for (const auto &[time_step, obstacle_ids] : relevant_obstacle_ids_over_time) {
        auto relevant_obstacles =
            env_model->get_world()->getObstacles() | std::views::filter([&obstacle_ids](const auto &obstacle) {
                return obstacle_ids.contains(obstacle->getId());
            });
        const auto &[ego_prio_min, ego_prio_max] =
            env_model->get_ego_approximations()->get_priority_range(time_step, ego_turn);
        for (const auto &obstacle : relevant_obstacles) {
            auto obs_prio = env_model->get_priority(time_step, obstacle, other_turn);
            if (!obs_prio.has_value()) {
                // Prediction for time step does not exist
                continue;
            }
            std::optional<bool> three_valued_result;
            switch (mode) {
            case PriorityMode::EGO_HAS_PRIORITY:
                three_valued_result = ego_has_prio(ego_prio_min, ego_prio_max, obs_prio.value());
                break;
            case PriorityMode::OTHER_HAS_PRIORITY:
                three_valued_result = other_has_prio(ego_prio_min, ego_prio_max, obs_prio.value());
                break;
            case PriorityMode::SAME_PRIORITY:
                three_valued_result = same_prio(ego_prio_min, ego_prio_max, obs_prio.value());
                break;
            }
            if (three_valued_result.has_value() && three_valued_result.value()) {
                true_false_obstacle_ids[time_step].first.emplace(obstacle->getId());
            } else if (three_valued_result.has_value() && !three_valued_result.value()) {
                true_false_obstacle_ids[time_step].second.emplace(obstacle->getId());
            }
        }
    }
    return true_false_obstacle_ids;
}

std::optional<bool> PriorityExtractor::ego_has_prio(int ego_prio_min, int ego_prio_max, int obs_prio) {
    constexpr int min_prio = std::numeric_limits<int>::min();
    if (ego_prio_min > obs_prio && ego_prio_min != min_prio && obs_prio != min_prio) {
        return true;
    } else if (ego_prio_max <= obs_prio || ego_prio_max == min_prio || obs_prio == min_prio) {
        return false;
    } else {
        return std::nullopt;
    }
}

std::optional<bool> PriorityExtractor::other_has_prio(int ego_prio_min, int ego_prio_max, int obs_prio) {
    constexpr int min_prio = std::numeric_limits<int>::min();
    if (obs_prio > ego_prio_max && ego_prio_min != min_prio && obs_prio != min_prio) {
        return true;
    } else if (obs_prio <= ego_prio_min || ego_prio_max == min_prio || obs_prio == min_prio) {
        return false;
    } else {
        return std::nullopt;
    }
}

std::optional<bool> PriorityExtractor::same_prio(int ego_prio_min, int ego_prio_max, int obs_prio) {
    auto ego_prio = ego_has_prio(ego_prio_min, ego_prio_max, obs_prio);
    if (ego_prio.has_value() && ego_prio.value()) {
        return false;
    }
    auto other_prio = other_has_prio(ego_prio_min, ego_prio_max, obs_prio);
    if (other_prio.has_value() && other_prio.value()) {
        return false;
    }
    if (ego_prio.has_value() && !ego_prio.value() && other_prio.has_value() && !other_prio.value()) {
        return true;
    }
    return std::nullopt;
}
