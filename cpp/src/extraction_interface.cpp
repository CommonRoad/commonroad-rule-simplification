#include "cr_knowledge_extraction/extraction_interface.hpp"
#include "cr_knowledge_extraction/proposition.hpp"
#include "cr_knowledge_extraction/relationship/equivalence/in_same_lane_equiv_extractor.hpp"
#include "cr_knowledge_extraction/relationship/implication/in_front_of_impl_extractor.hpp"

#include <ranges>
#include <unordered_set>

using namespace knowledge_extraction;

std::unordered_map<time_step_t, ExtractionResult>
ExtractionInterface::extract(const std::unordered_map<time_step_t, std::vector<std::string>> &relevant_propositions) {
    std::unordered_map<Proposition, std::unordered_map<time_step_t, std::unordered_set<size_t>>>
        relevant_obstacles_over_time_per_prop{};
    for (const auto &[time_step, propositions] : relevant_propositions) {
        for (const auto &prop : propositions) {
            auto [prop_enum, param] = proposition::from_string(prop);
            // Touch proposition at time step to create an empty set
            // (needed for propositions that do not have parameters)
            relevant_obstacles_over_time_per_prop[prop_enum][time_step];
            if (param.has_value()) {
                relevant_obstacles_over_time_per_prop[prop_enum][time_step].insert(param.value());
            }
        }
    }

    std::unordered_map<time_step_t, ExtractionResult> result{};
    // Kleene extraction
    for (const auto &[prop, relevant_obstacles_over_time] : relevant_obstacles_over_time_per_prop) {
        auto extractor = create_kleene_extractor(prop);
        if (extractor.has_value()) {
            auto relationships = extractor.value()->extract(relevant_obstacles_over_time);
            for (auto &[time_step, positiveNegative] : relationships) {
                std::ranges::move(positiveNegative.first | std::views::transform([&prop](const auto &obstacle_id) {
                                      return proposition::to_string(prop, obstacle_id);
                                  }),
                                  std::back_inserter(result[time_step].positive_propositions));
                std::ranges::move(positiveNegative.second | std::views::transform([&prop](const auto &obstacle_id) {
                                      return proposition::to_string(prop, obstacle_id);
                                  }),
                                  std::back_inserter(result[time_step].negative_propositions));
            }
        }
    }

    // Relationship extraction
    for (const auto &[prop, relevant_obstacles_over_time] : relevant_obstacles_over_time_per_prop) {
        auto extractor = create_relationship_extractor(prop);
        if (extractor.has_value()) {
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

    return result;
}

std::optional<std::unique_ptr<kleene::KleeneExtractor>> ExtractionInterface::create_kleene_extractor(Proposition prop) {
    switch (prop) {
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
