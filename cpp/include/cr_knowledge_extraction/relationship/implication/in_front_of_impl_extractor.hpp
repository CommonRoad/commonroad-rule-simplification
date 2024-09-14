#pragma once

#include <utility>

#include "cr_knowledge_extraction/relationship/relationship_extractor.hpp"

namespace knowledge_extraction::relationship::implication {
class InFrontOfImplExtractor : public RelationshipExtractor {
  public:
    InFrontOfImplExtractor(std::shared_ptr<knowledge_extraction::env_model::EnvironmentModel> env_model)
        : RelationshipExtractor(std::move(env_model), Proposition::IN_FRONT_OF, Proposition::IN_FRONT_OF,
                                RelationshipType::IMPLICATION){};

    std::unordered_map<time_step_t, std::vector<Relationship>>
    extract(const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>>
                &relevant_obstacle_ids_over_time) const override;
};
} // namespace knowledge_extraction::relationship::implication
