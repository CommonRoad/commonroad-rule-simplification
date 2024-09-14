#pragma once

#include "cr_knowledge_extraction/ego_behavior/behavior_overapproximation.hpp"
#include "cr_knowledge_extraction/kleene/kleene_extractor.hpp"

namespace knowledge_extraction::kleene::position {
class OnLaneletWithTypeExtractor : public KleeneExtractor {
  private:
    const LaneletType lanelet_type;

  public:
    OnLaneletWithTypeExtractor(std::shared_ptr<knowledge_extraction::env_model::EnvironmentModel> env_model,
                               Proposition proposition, LaneletType lanelet_type)
        : KleeneExtractor(std::move(env_model), proposition), lanelet_type(lanelet_type) {}

    std::unordered_map<time_step_t, TrueFalseObstacleIds>
    extract(const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>>
                &relevant_obstacle_ids_over_time) const override;
};
} // namespace knowledge_extraction::kleene::position
