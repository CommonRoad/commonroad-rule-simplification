#pragma once

#include "cr_knowledge_extraction/kleene/kleene_extractor.hpp"

namespace knowledge_extraction::kleene::position {
class InSameLaneExtractor : public KleeneExtractor {
  public:
    InSameLaneExtractor(std::shared_ptr<knowledge_extraction::env_model::EnvironmentModel> env_model)
        : KleeneExtractor(std::move(env_model), Proposition::IN_SAME_LANE) {}

    std::unordered_map<time_step_t, TrueFalseObstacleIds>
    extract(const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>>
                &relevant_obstacle_ids_over_time) const override;
};
} // namespace knowledge_extraction::kleene::position
