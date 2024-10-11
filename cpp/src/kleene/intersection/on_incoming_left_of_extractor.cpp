#include "cr_knowledge_extraction/kleene/intersection/on_incoming_left_of_extractor.hpp"

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>

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

        auto left_of_incomings_could = get_incoming_left_of_ids_from_lanelets(
            env_model->get_ego_approximations()->get_covered_lanelets(time_step), road_network);

        auto left_of_incomings_must = get_incoming_left_of_ids_from_lanelets(
            env_model->get_ego_approximations()->get_intersected_lanelets(time_step), road_network);

        for (const auto &obstacle : relevant_obstacles) {
            auto is_left_of =
                is_on_incoming_left_of(time_step, obstacle, left_of_incomings_could, left_of_incomings_must);
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

std::optional<bool>
OnIncomingLeftOfExtractor::is_on_incoming_left_of(const time_step_t &time_step,
                                                  const std::shared_ptr<Obstacle> &obstacle,
                                                  const std::unordered_set<size_t> &left_of_incomings_could,
                                                  const std::unordered_set<size_t> &left_of_incomings_must) const {
    if (left_of_incomings_could.empty()) {
        return false;
    }

    const auto &road_network = env_model->get_world()->getRoadNetwork();

    std::vector<std::shared_ptr<Lanelet>> obs_lanelets;
    try {
        obs_lanelets = obstacle->getOccupiedLaneletsByShape(road_network, time_step);
    } catch (const std::logic_error &e) {
        return std::nullopt;
    }
    if (std::ranges::none_of(obs_lanelets,
                             [](const auto &lanelet) { return lanelet->hasLaneletType(LaneletType::incoming); })) {
        return false;
    }

    auto obs_incomings_ =
        obstacle->getReferenceLane(road_network, time_step)->getContainedLanelets() |
        std::views::filter([](const auto &lanelet) { return lanelet->hasLaneletType(LaneletType::incoming); }) |
        std::views::transform([&road_network](const auto &lanelet) {
            auto incoming = road_network->findIncomingGroupByLanelet(lanelet);
            if (!incoming) {
                throw std::runtime_error{"missing incoming (obstacle)"};
            }
            return incoming->getId();
        });
    std::unordered_set<size_t> obs_incomings{obs_incomings_.begin(), obs_incomings_.end()};

    if (std::ranges::none_of(left_of_incomings_could, [&obs_incomings](const auto &left_of_incoming_id) {
            return obs_incomings.contains(left_of_incoming_id);
        })) {
        return false;
    }

    if (std::ranges::all_of(left_of_incomings_must, [&obs_incomings](const auto &left_of_incoming_id) {
            return obs_incomings.contains(left_of_incoming_id);
        })) {
        return true;
    }

    return std::nullopt;
}

std::unordered_set<size_t>
OnIncomingLeftOfExtractor::get_incoming_left_of_ids_from_lanelets(const std::vector<std::shared_ptr<Lanelet>> &lanelets,
                                                                  const std::shared_ptr<RoadNetwork> &road_network) {
    auto left_of_ids =
        lanelets |
        std::views::filter([](const auto &lanelet) { return lanelet->hasLaneletType(LaneletType::incoming); }) |
        std::views::transform([&road_network](const auto &lanelet) {
            auto incoming = road_network->findIncomingGroupByLanelet(lanelet);
            if (!incoming) {
                throw std::runtime_error{"missing incoming (ego)"};
            }
            if (!incoming->getIsLeftOf()) {
                throw std::runtime_error{"missing 'left of' incoming"};
            }
            return incoming->getIsLeftOf()->getId();
        });
    return {left_of_ids.begin(), left_of_ids.end()};
}
