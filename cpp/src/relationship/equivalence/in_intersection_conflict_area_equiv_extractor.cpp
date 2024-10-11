#include "cr_knowledge_extraction/relationship/equivalence/in_intersection_conflict_area_equiv_extractor.hpp"

#include <boost/functional/hash.hpp>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>

#include <algorithm>
#include <ranges>

using namespace knowledge_extraction::relationship::equivalence;

std::unordered_map<time_step_t, std::vector<InIntersectionConflictAreaEquivExtractor::Relationship>>
InIntersectionConflictAreaEquivExtractor::extract(
    const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>> &relevant_obstacle_ids_over_time)
    const {
    std::unordered_map<time_step_t, std::vector<Relationship>> result;

    Obstacle obs;

    for (const auto &[time_step, obstacle_ids] : relevant_obstacle_ids_over_time) {
        auto relevant_obstacle_lanelets_ =
            env_model->get_world()->getObstacles() | std::views::filter([&obstacle_ids](const auto &obstacle) {
                return obstacle_ids.contains(obstacle->getId());
            }) |
            std::views::transform([this, &time_step](const auto &obstacle) {
                try {
                    const auto &ref_path_lanelets =
                        obstacle->getReferenceLane(env_model->get_world()->getRoadNetwork(), time_step)
                            ->getContainedLanelets();
                    auto intersection_lanelet_ids =
                        ref_path_lanelets | std::views::filter([](const auto &lanelet) {
                            return lanelet->hasLaneletType(LaneletType::intersection);
                        }) |
                        std::views::transform([](const auto &lanelet) { return lanelet->getId(); });
                    return std::make_pair(obstacle->getId(),
                                          std::optional{std::set<size_t>{intersection_lanelet_ids.begin(),
                                                                         intersection_lanelet_ids.end()}});
                } catch (const std::logic_error &e) {
                    return std::make_pair(obstacle->getId(), std::optional<std::set<size_t>>{});
                }
            }) |
            std::views::filter([](const auto &pair) { return pair.second.has_value(); }) |
            std::views::transform([](const auto &pair) { return std::make_pair(pair.first, pair.second.value()); });
        std::vector<std::pair<size_t, std::set<size_t>>> relevant_obstacle_lanelets{relevant_obstacle_lanelets_.begin(),
                                                                                    relevant_obstacle_lanelets_.end()};

        std::unordered_map<std::set<size_t>, std::vector<size_t>, boost::hash<std::set<size_t>>> equivalence_classes{};
        for (auto &[obstacle_id, lanelet_ids] : relevant_obstacle_lanelets) {
            equivalence_classes[std::move(lanelet_ids)].emplace_back(obstacle_id);
        }

        // We create (size of class - 1) equivalences per equivalence class and all obstacles have been put in a class
        result[time_step].reserve(relevant_obstacle_lanelets.size() - equivalence_classes.size());
        for (const auto &[_, eq_class] : equivalence_classes) {
            for (size_t i = 0; i < eq_class.size() - 1; ++i) {
                result[time_step].emplace_back(RelationshipType::EQUIVALENCE, eq_class[i], eq_class[i + 1]);
            }
        }
    }
    return result;
}
