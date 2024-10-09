#include "cr_knowledge_extraction/kleene/intersection/on_incoming_left_of_extractor.hpp"

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/road_network.h>

#include <ranges>

using namespace knowledge_extraction::kleene::intersection;

std::unordered_map<time_step_t, OnIncomingLeftOfExtractor::TrueFalseObstacleIds> OnIncomingLeftOfExtractor::extract(
    const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>> &relevant_obstacle_ids_over_time)
    const {
    const auto &road_network = env_model->get_world()->getRoadNetwork();

    std::unordered_map<time_step_t, TrueFalseObstacleIds> true_false_obstacle_ids;
    for (const auto &[time_step, obstacle_ids] : relevant_obstacle_ids_over_time) {
        auto relevant_obstacles =
            env_model->get_world()->getObstacles() | std::views::filter([&obstacle_ids](const auto &obstacle) {
                return obstacle_ids.contains(obstacle->getId());
            });

        auto ego_incomings_could_ = env_model->get_ego_approximations()->get_covered_lanelets(time_step) |
                                    std::views::filter([](const auto &lanelet) {
                                        return lanelet->lanelet->hasLaneletType(LaneletType::incoming);
                                    }) |
                                    std::views::transform([&road_network](const auto &lanelet) {
                                        auto incoming = road_network->findIncomingGroupByLanelet(lanelet->lanelet);
                                        if (!incoming) {
                                            throw std::runtime_error{"missing incoming (ego)"};
                                        }
                                        return incoming->getId();
                                    });
        std::unordered_set<size_t> ego_incomings_could{ego_incomings_could_.begin(), ego_incomings_could_.end()};

        auto ego_incomings_must_ = env_model->get_ego_approximations()->get_intersected_lanelets(time_step) |
                                   std::views::filter([](const auto &lanelet) {
                                       return lanelet->lanelet->hasLaneletType(LaneletType::incoming);
                                   }) |
                                   std::views::transform([&road_network](const auto &lanelet) {
                                       auto incoming = road_network->findIncomingGroupByLanelet(lanelet->lanelet);
                                       if (!incoming) {
                                           throw std::runtime_error{"missing incoming (ego)"};
                                       }
                                       return incoming->getId();
                                   });
        std::unordered_set<size_t> ego_incomings_must{ego_incomings_must_.begin(), ego_incomings_must_.end()};

        for (const auto &obstacle : relevant_obstacles) {
            auto is_left_of = is_on_incoming_left_of(time_step, obstacle, ego_incomings_could, ego_incomings_must);
            if (is_left_of.has_value()) {
                if (is_left_of.value()) {
                    true_false_obstacle_ids[time_step].first.emplace(obstacle->getId());
                } else {
                    true_false_obstacle_ids[time_step].second.emplace(obstacle->getId());
                }
            }
        }
    }
    return true_false_obstacle_ids;
}

std::optional<bool> OnIncomingLeftOfExtractor::is_on_incoming_left_of(
    const time_step_t &time_step, const std::shared_ptr<Obstacle> &obstacle,
    const std::unordered_set<size_t> &ego_incomings_could, const std::unordered_set<size_t> &ego_incomings_must) const {
    const auto &road_network = env_model->get_world()->getRoadNetwork();

    auto obs_lanelets = obstacle->getOccupiedLaneletsByShape(road_network, time_step);
    if (std::ranges::none_of(obs_lanelets,
                             [](const auto &lanelet) { return lanelet->hasLaneletType(LaneletType::incoming); })) {
        return false;
    }
    for (const auto &lanelet : obstacle->getReferenceLane(road_network, time_step)->getContainedLanelets()) {
        if (!lanelet->hasLaneletType(LaneletType::incoming)) {
            continue;
        }
        auto incoming = road_network->findIncomingGroupByLanelet(lanelet);
        if (!incoming) {
            throw std::runtime_error{"missing incoming (obstacle)"};
        }
        if (!incoming->getIsLeftOf()) {
            throw std::runtime_error{"missing 'left of' incoming"};
        }
        auto left_of = incoming->getIsLeftOf();
        if (!ego_incomings_could.contains(left_of->getId())) {
            return false;
        }
        if (ego_incomings_must.contains(left_of->getId())) {
            return true;
        }
    }
    return std::nullopt;
}
