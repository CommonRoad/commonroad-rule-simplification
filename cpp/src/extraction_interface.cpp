#include "cr_knowledge_extraction/extraction_interface.hpp"

#include "cr_knowledge_extraction/kleene/braking/safe_distance_extractor.hpp"
#include "cr_knowledge_extraction/kleene/ego_independent/ego_independent_extractor.hpp"
#include "cr_knowledge_extraction/kleene/general/cut_in_extractor.hpp"
#include "cr_knowledge_extraction/kleene/intersection/on_incoming_left_of_extractor.hpp"
#include "cr_knowledge_extraction/kleene/intersection/other_turning_extractor.hpp"
#include "cr_knowledge_extraction/kleene/position/at_traffic_sign_extractor.hpp"
#include "cr_knowledge_extraction/kleene/position/in_front_of_extractor.hpp"
#include "cr_knowledge_extraction/kleene/position/in_same_lane_extractor.hpp"
#include "cr_knowledge_extraction/kleene/position/on_lanelet_with_type_extractor.hpp"
#include "cr_knowledge_extraction/kleene/position/on_main_carriageway_right_lane_extractor.hpp"
#include "cr_knowledge_extraction/kleene/position/relevant_traffic_light_extractor.hpp"
#include "cr_knowledge_extraction/kleene/regulatory/priority_extractor.hpp"
#include "cr_knowledge_extraction/proposition.hpp"
#include "cr_knowledge_extraction/relationship/equivalence/in_intersection_conflict_area_equiv_extractor.hpp"
#include "cr_knowledge_extraction/relationship/equivalence/in_same_lane_equiv_extractor.hpp"
#include "cr_knowledge_extraction/relationship/implication/in_front_of_impl_extractor.hpp"
#include "cr_knowledge_extraction/relationship/implication/safe_distance_impl_extractor.hpp"
#include "cr_knowledge_extraction/road_network/curvilinear_road_network.hpp"

#include <commonroad_cpp/predicates/lane/on_lanelet_with_type_predicate.h>

#include <spdlog/spdlog.h>

#include <ranges>
#include <unordered_set>

using namespace knowledge_extraction;

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

std::unordered_map<time_step_t, ExtractionResult> ExtractionInterface::extract_all_but_implications(
    const std::unordered_map<time_step_t, std::vector<std::string>> &relevant_propositions) {
    auto relevant_obstacles = compute_relevant_obstacles(relevant_propositions);

    std::unordered_map<time_step_t, ExtractionResult> result{};
    // Kleene extraction
    extract_kleene(relevant_obstacles, result);
    // Relationship extraction
    extract_relationships(relevant_obstacles, result, relationship::RelationshipType::EQUIVALENCE);

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
    case Proposition::ON_MAIN_CARRIAGEWAY:
        return std::make_unique<kleene::position::OnLaneletWithTypeExtractor>(env_model, prop,
                                                                              LaneletType::mainCarriageWay);
    case Proposition::IN_INTERSECTION:
        return std::make_unique<kleene::position::OnLaneletWithTypeExtractor>(env_model, prop,
                                                                              LaneletType::intersection);
    case Proposition::ON_MAIN_CARRIAGEWAY_RIGHT_LANE:
        return std::make_unique<kleene::position::OnMainCarriagewayRightLaneExtractor>(env_model);
    case Proposition::IN_FRONT_OF:
        return std::make_unique<kleene::position::InFrontOfExtractor>(env_model);
    case Proposition::IN_SAME_LANE:
        return std::make_unique<kleene::position::InSameLaneExtractor>(env_model);
    case Proposition::CUT_IN:
        return std::make_unique<kleene::general::CutInExtractor>(env_model);
    case Proposition::KEEPS_SAFE_DISTANCE_PREC:
        return std::make_unique<kleene::braking::SafeDistanceExtractor>(env_model);
    case Proposition::OTHER_ON_ACCESS_RAMP:
        return std::make_unique<kleene::ego_independent::EgoIndependentExtractor>(
            env_model, prop, std::make_unique<OnLaneletWithTypePredicate>(),
            std::vector{lanelet_type_to_string(LaneletType::accessRamp)});
    case Proposition::OTHER_ON_MAIN_CARRIAGEWAY:
        return std::make_unique<kleene::ego_independent::EgoIndependentExtractor>(
            env_model, prop, std::make_unique<OnLaneletWithTypePredicate>(),
            std::vector{lanelet_type_to_string(LaneletType::mainCarriageWay)});
    case Proposition::RELEVANT_TRAFFIC_LIGHT:
        return std::make_unique<kleene::position::RelevantTrafficLightExtractor>(env_model);
    case Proposition::AT_STOP_SIGN:
        return std::make_unique<kleene::position::AtTrafficSignExtractor>(env_model, prop, TrafficSignTypes::STOP);
    case Proposition::ON_INCOMING_LEFT_OF:
        return std::make_unique<kleene::intersection::OnIncomingLeftOfExtractor>(env_model);
    case Proposition::OTHER_TURNING_LEFT:
        return std::make_unique<kleene::intersection::OtherTurningExtractor>(env_model, prop, Direction::left);
    case Proposition::OTHER_GOING_STRAIGHT:
        return std::make_unique<kleene::intersection::OtherTurningExtractor>(env_model, prop, Direction::straight);
    case Proposition::OTHER_TURNING_RIGHT:
        return std::make_unique<kleene::intersection::OtherTurningExtractor>(env_model, prop, Direction::right);

    case Proposition::SAME_LEFT_LEFT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::SAME_PRIORITY, Direction::left,
            Direction::left);
    case Proposition::SAME_LEFT_STRAIGHT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::SAME_PRIORITY, Direction::left,
            Direction::straight);
    case Proposition::SAME_LEFT_RIGHT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::SAME_PRIORITY, Direction::left,
            Direction::right);
    case Proposition::SAME_STRAIGHT_LEFT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::SAME_PRIORITY, Direction::straight,
            Direction::left);
    case Proposition::SAME_STRAIGHT_STRAIGHT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::SAME_PRIORITY, Direction::straight,
            Direction::straight);
    case Proposition::SAME_STRAIGHT_RIGHT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::SAME_PRIORITY, Direction::straight,
            Direction::right);
    case Proposition::SAME_RIGHT_LEFT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::SAME_PRIORITY, Direction::right,
            Direction::left);
    case Proposition::SAME_RIGHT_STRAIGHT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::SAME_PRIORITY, Direction::right,
            Direction::straight);
    case Proposition::SAME_RIGHT_RIGHT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::SAME_PRIORITY, Direction::right,
            Direction::right);

    case Proposition::HAS_LEFT_LEFT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::EGO_HAS_PRIORITY, Direction::left,
            Direction::left);
    case Proposition::HAS_LEFT_STRAIGHT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::EGO_HAS_PRIORITY, Direction::left,
            Direction::straight);
    case Proposition::HAS_LEFT_RIGHT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::EGO_HAS_PRIORITY, Direction::left,
            Direction::right);
    case Proposition::HAS_STRAIGHT_LEFT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::EGO_HAS_PRIORITY, Direction::straight,
            Direction::left);
    case Proposition::HAS_STRAIGHT_STRAIGHT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::EGO_HAS_PRIORITY, Direction::straight,
            Direction::straight);
    case Proposition::HAS_STRAIGHT_RIGHT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::EGO_HAS_PRIORITY, Direction::straight,
            Direction::right);
    case Proposition::HAS_RIGHT_LEFT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::EGO_HAS_PRIORITY, Direction::right,
            Direction::left);
    case Proposition::HAS_RIGHT_STRAIGHT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::EGO_HAS_PRIORITY, Direction::right,
            Direction::straight);
    case Proposition::HAS_RIGHT_RIGHT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::EGO_HAS_PRIORITY, Direction::right,
            Direction::right);

    case Proposition::OTHER_HAS_LEFT_LEFT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::OTHER_HAS_PRIORITY, Direction::left,
            Direction::left);
    case Proposition::OTHER_HAS_LEFT_STRAIGHT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::OTHER_HAS_PRIORITY, Direction::left,
            Direction::straight);
    case Proposition::OTHER_HAS_LEFT_RIGHT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::OTHER_HAS_PRIORITY, Direction::left,
            Direction::right);
    case Proposition::OTHER_HAS_STRAIGHT_LEFT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::OTHER_HAS_PRIORITY,
            Direction::straight, Direction::left);
    case Proposition::OTHER_HAS_STRAIGHT_STRAIGHT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::OTHER_HAS_PRIORITY,
            Direction::straight, Direction::straight);
    case Proposition::OTHER_HAS_STRAIGHT_RIGHT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::OTHER_HAS_PRIORITY,
            Direction::straight, Direction::right);
    case Proposition::OTHER_HAS_RIGHT_LEFT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::OTHER_HAS_PRIORITY, Direction::right,
            Direction::left);
    case Proposition::OTHER_HAS_RIGHT_STRAIGHT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::OTHER_HAS_PRIORITY, Direction::right,
            Direction::straight);
    case Proposition::OTHER_HAS_RIGHT_RIGHT_PRIORITY:
        return std::make_unique<kleene::regulatory::PriorityExtractor>(
            env_model, prop, kleene::regulatory::PriorityExtractor::PriorityMode::OTHER_HAS_PRIORITY, Direction::right,
            Direction::right);

    default:
        return std::nullopt;
    }
}

std::optional<std::unique_ptr<relationship::RelationshipExtractor>>
ExtractionInterface::create_relationship_extractor(Proposition prop) {
    switch (prop) {
    case Proposition::IN_SAME_LANE:
        return std::make_unique<relationship::equivalence::InSameLaneEquivExtractor>(env_model);
    case Proposition::IN_INTERSECTION_CONFLICT_AREA:
        return std::make_unique<relationship::equivalence::InIntersectionConflictAreaEquivExtractor>(env_model);
    case Proposition::IN_FRONT_OF:
        return std::make_unique<relationship::implication::InFrontOfImplExtractor>(env_model);
    case Proposition::KEEPS_SAFE_DISTANCE_PREC:
        return std::make_unique<relationship::implication::SafeDistanceImplExtractor>(env_model);
    default:
        return std::nullopt;
    }
}

ExtractionInterface::RelevantObstacles ExtractionInterface::compute_relevant_obstacles(
    const std::unordered_map<time_step_t, std::vector<std::string>> &relevant_propositions) {
    RelevantObstacles relevant_obstacles_over_time_per_prop{};
    for (const auto &[time_step, propositions] : relevant_propositions) {
        for (const auto &prop : propositions) {
            try {
                auto [prop_enum, param] = proposition::from_string(prop);
                relevant_obstacles_over_time_per_prop[prop_enum][time_step].insert(param);
            } catch (const std::logic_error &e) {
                // Unknown propositions are simply ignored with a warning
                spdlog::warn("Unknown proposition: {}. No knowledge will be extracted for this proposition!", prop);
                continue;
            }
        }
    }
    return relevant_obstacles_over_time_per_prop;
}
