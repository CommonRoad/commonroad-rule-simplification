#include "cr_knowledge_extraction/extraction_interface.hpp"
#include "cr_knowledge_extraction/kleene/ego_independent/ego_independent_extractor.hpp"
#include "cr_knowledge_extraction/proposition.hpp"
#include "cr_knowledge_extraction/relationship/equivalence/in_same_lane_equiv_extractor.hpp"
#include "cr_knowledge_extraction/relationship/implication/in_front_of_impl_extractor.hpp"
#include "cr_knowledge_extraction/road_network/curvilinear_road_network.hpp"

#include <commonroad_cpp/geometry/geometric_operations.h>
#include <commonroad_cpp/predicates/lane/on_lanelet_with_type_predicate.h>

#include <spdlog/spdlog.h>

#include <ranges>
#include <unordered_set>

using namespace knowledge_extraction;

std::shared_ptr<ego_behavior::BehaviorOverapproximation>
ExtractionInterface::make_ego_approximations(const std::shared_ptr<World> &world,
                                             const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs,
                                             ego_behavior::EgoParameters ego_params) {
    auto &initial_state = ego_params.initial_state;

    auto ccs_pos = ego_ccs->convertToCurvilinearCoords(initial_state.getXPosition(), initial_state.getYPosition());
    initial_state.setLonPosition(ccs_pos.x());
    initial_state.setLatPosition(ccs_pos.y());

    auto ccs_tangent = ego_ccs->tangent(ccs_pos.x());
    auto ccs_orientation = std::atan2(ccs_tangent.y(), ccs_tangent.x());
    auto theta = geometric_operations::subtractOrientations(initial_state.getGlobalOrientation(), ccs_orientation);
    initial_state.setCurvilinearOrientation(theta);

    auto dt = world->getDt();

    return std::make_shared<ego_behavior::BehaviorOverapproximation>(
        dt, ego_params, road_network::CurvilinearRoadNetwork{world, ego_ccs});
}

std::unordered_map<time_step_t, ExtractionResult> ExtractionInterface::extract_all(
    const std::unordered_map<time_step_t, std::vector<std::string>> &relevant_propositions) {
    auto relevant_obstacles = compute_relevant_obstacles(relevant_propositions);

    std::unordered_map<time_step_t, ExtractionResult> result{};
    // Kleene extraction
    extract_kleene(relevant_obstacles, result);
    // Relationship extraction
    extract_relationships(relevant_obstacles, result);

    return result;
}

std::unordered_map<time_step_t, ExtractionResult> ExtractionInterface::extract_kleene(
    const std::unordered_map<time_step_t, std::vector<std::string>> &relevant_propositions) {
    auto relevant_obstacles = compute_relevant_obstacles(relevant_propositions);
    std::unordered_map<time_step_t, ExtractionResult> result{};
    extract_kleene(relevant_obstacles, result);
    return result;
}

void ExtractionInterface::extract_kleene(const RelevantObstacles &relevant_obstacles,
                                         std::unordered_map<time_step_t, ExtractionResult> &result) {
    for (const auto &[prop, relevant_obstacles_over_time] : relevant_obstacles) {
        auto extractor = create_kleene_extractor(prop);
        if (extractor.has_value()) {
            auto kleene_values = extractor.value()->extract(relevant_obstacles_over_time);
            for (auto &[time_step, positive_negative] : kleene_values) {
                std::ranges::move(positive_negative.first | std::views::transform([&prop](const auto &obstacle_id) {
                                      return proposition::to_string(prop, obstacle_id);
                                  }),
                                  std::back_inserter(result[time_step].positive_propositions));
                std::ranges::move(positive_negative.second | std::views::transform([&prop](const auto &obstacle_id) {
                                      return proposition::to_string(prop, obstacle_id);
                                  }),
                                  std::back_inserter(result[time_step].negative_propositions));
            }
        }
    }
}

std::unordered_map<time_step_t, ExtractionResult> ExtractionInterface::extract_relationships(
    const std::unordered_map<time_step_t, std::vector<std::string>> &relevant_propositions) {
    auto relevant_obstacles = compute_relevant_obstacles(relevant_propositions);
    std::unordered_map<time_step_t, ExtractionResult> result{};
    extract_relationships(relevant_obstacles, result);
    return result;
}

std::unordered_map<time_step_t, ExtractionResult> ExtractionInterface::extract_equivalences(
    const std::unordered_map<time_step_t, std::vector<std::string>> &relevant_propositions) {
    auto relevant_obstacles = compute_relevant_obstacles(relevant_propositions);
    std::unordered_map<time_step_t, ExtractionResult> result{};
    extract_relationships(relevant_obstacles, result, relationship::RelationshipType::EQUIVALENCE);
    return result;
}

std::unordered_map<time_step_t, ExtractionResult> ExtractionInterface::extract_implications(
    const std::unordered_map<time_step_t, std::vector<std::string>> &relevant_propositions) {
    auto relevant_obstacles = compute_relevant_obstacles(relevant_propositions);
    std::unordered_map<time_step_t, ExtractionResult> result{};
    extract_relationships(relevant_obstacles, result, relationship::RelationshipType::IMPLICATION);
    return result;
}

void ExtractionInterface::extract_relationships(const ExtractionInterface::RelevantObstacles &relevant_obstacles,
                                                std::unordered_map<time_step_t, ExtractionResult> &result,
                                                std::optional<relationship::RelationshipType> type) {
    for (const auto &[prop, relevant_obstacles_over_time] : relevant_obstacles) {
        auto extractor = create_relationship_extractor(prop);
        if (extractor.has_value()) {
            if (type.has_value() && extractor.value()->get_dominant_relationship() != type.value()) {
                continue;
            }
            auto [lhs, rhs] = extractor.value()->get_propositions();
            auto relationships = extractor.value()->extract(relevant_obstacles_over_time);
            for (auto &[time_step, relations] : relationships) {
                for (const auto &rel : relations) {
                    switch (std::get<0>(rel)) {
                    case relationship::RelationshipType::IMPLICATION:
                        result[time_step].implications.emplace_back(proposition::to_string(lhs, std::get<1>(rel)),
                                                                    proposition::to_string(rhs, std::get<2>(rel)));
                        break;
                    case relationship::RelationshipType::EQUIVALENCE:
                        result[time_step].equivalences.emplace_back(proposition::to_string(lhs, std::get<1>(rel)),
                                                                    proposition::to_string(rhs, std::get<2>(rel)));
                        break;
                    default:
                        break;
                    }
                }
            }
        }
    }
}

auto lanelet_type_to_string(LaneletType lanelet_type) {
    return std::ranges::find_if(LaneletTypeNames,
                                [&lanelet_type](const auto &pair) { return pair.second == lanelet_type; })
        ->first;
}

std::optional<std::unique_ptr<kleene::KleeneExtractor>> ExtractionInterface::create_kleene_extractor(Proposition prop) {
    switch (prop) {
    case Proposition::OTHER_ON_ACCESS_RAMP:
        return std::make_unique<kleene::ego_independent::EgoIndependentExtractor>(
            world, ego_ccs, prop, std::make_unique<OnLaneletWithTypePredicate>(),
            std::vector{lanelet_type_to_string(LaneletType::accessRamp)});
    case Proposition::OTHER_ON_MAIN_CARRIAGEWAY:
        return std::make_unique<kleene::ego_independent::EgoIndependentExtractor>(
            world, ego_ccs, prop, std::make_unique<OnLaneletWithTypePredicate>(),
            std::vector{lanelet_type_to_string(LaneletType::mainCarriageWay)});
    default:
        return std::nullopt;
    }
}

std::optional<std::unique_ptr<relationship::RelationshipExtractor>>
ExtractionInterface::create_relationship_extractor(Proposition prop) {
    switch (prop) {
    case Proposition::IN_SAME_LANE:
        return std::make_unique<relationship::equivalence::InSameLaneEquivExtractor>(world, ego_ccs);
    case Proposition::IN_FRONT_OF:
        return std::make_unique<relationship::implication::InFrontOfImplExtractor>(world, ego_ccs);
    default:
        return std::nullopt;
    }
}

ExtractionInterface::RelevantObstacles ExtractionInterface::compute_relevant_obstacles(
    const std::unordered_map<time_step_t, std::vector<std::string>> &relevant_propositions) {
    std::unordered_map<Proposition, std::unordered_map<time_step_t, std::unordered_set<size_t>>>
        relevant_obstacles_over_time_per_prop{};
    for (const auto &[time_step, propositions] : relevant_propositions) {
        for (const auto &prop : propositions) {
            try {
                auto [prop_enum, param] = proposition::from_string(prop);
                // Touch proposition at time step to create an empty set
                // (needed for propositions that do not have parameters)
                relevant_obstacles_over_time_per_prop[prop_enum][time_step];
                if (param.has_value()) {
                    relevant_obstacles_over_time_per_prop[prop_enum][time_step].insert(param.value());
                }
            } catch (const std::logic_error &e) {
                // Unknown propositions are simply ignored with a warning
                spdlog::warn("Unknown proposition: {}. No knowledge will be extracted for this proposition!", prop);
                continue;
            }
        }
    }
    return relevant_obstacles_over_time_per_prop;
}
