#pragma once

#include "cr_knowledge_extraction/kleene/kleene_extractor.hpp"

namespace knowledge_extraction::kleene::intersection {
class OnIncomingLeftOfExtractor : public KleeneExtractor {
  public:
    OnIncomingLeftOfExtractor(std::shared_ptr<knowledge_extraction::env_model::EnvironmentModel> env_model)
        : KleeneExtractor(std::move(env_model), Proposition::ON_INCOMING_LEFT_OF) {}

    std::unordered_map<time_step_t, TrueFalseObstacleIds>
    extract(const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>>
                &relevant_obstacle_ids_over_time) const override;
    std::optional<bool> is_on_incoming_left_of(const time_step_t &time_step, const std::shared_ptr<Obstacle> &obstacle,
                                               const std::unordered_set<size_t> &ego_incomings_could,
                                               const std::unordered_set<size_t> &ego_incomings_must) const;
};
} // namespace knowledge_extraction::kleene::intersection
