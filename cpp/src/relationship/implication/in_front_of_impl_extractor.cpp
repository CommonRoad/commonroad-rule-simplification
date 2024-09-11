#include "cr_knowledge_extraction/relationship/implication/in_front_of_impl_extractor.hpp"

#include <commonroad_cpp/obstacle/obstacle.h>

#include <algorithm>
#include <ranges>

using namespace knowledge_extraction::relationship::implication;

std::optional<double> InFrontOfImplExtractor::get_obstacle_rear(size_t time_step,
                                                                const std::shared_ptr<Obstacle> &obstacle) const {
    std::shared_ptr<State> obstacle_state;
    try {
        obstacle_state = obstacle->getStateByTimeStep(time_step);
    } catch (std::logic_error &e) {
        return std::nullopt;
    }
    if (!ego_ccs->cartesianPointInProjectionDomain(obstacle_state->getXPosition(), obstacle_state->getYPosition())) {
        return std::nullopt;
    }

    return obstacle->rearS(time_step, ego_ccs);
}

std::unordered_map<time_step_t, std::vector<InFrontOfImplExtractor::Relationship>> InFrontOfImplExtractor::extract(
    const std::unordered_map<time_step_t, std::unordered_set<size_t>> &relevant_obstacle_ids_over_time) const {
    std::unordered_map<time_step_t, std::vector<Relationship>> result;

    for (const auto &[time_step, obstacle_ids] : relevant_obstacle_ids_over_time) {
        auto relevant_obstacle_rears_ =
            world->getObstacles() | std::views::filter([&obstacle_ids](const auto &obstacle) {
                return obstacle_ids.contains(obstacle->getId());
            }) |
            std::views::transform([this, &time_step](const auto &obstacle) {
                return std::make_pair(obstacle->getId(), get_obstacle_rear(time_step, obstacle));
            }) |
            std::views::filter([](const auto &pair) { return pair.second.has_value(); }) |
            std::views::transform([](const auto &pair) { return std::make_pair(pair.first, pair.second.value()); });
        std::vector<std::pair<size_t, double>> relevant_obstacle_rears{relevant_obstacle_rears_.begin(),
                                                                       relevant_obstacle_rears_.end()};

        std::ranges::sort(relevant_obstacle_rears,
                          [](const auto &lhs, const auto &rhs) { return lhs.second < rhs.second; });

        for (size_t i = 0; i < relevant_obstacle_rears.size() - 1; ++i) {
            result[time_step].emplace_back(RelationshipType::IMPLICATION, relevant_obstacle_rears[i].first,
                                           relevant_obstacle_rears[i + 1].first);
        }
    }
    return result;
}
