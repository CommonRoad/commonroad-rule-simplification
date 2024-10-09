#pragma once

#include "cr_knowledge_extraction/kleene/kleene_extractor.hpp"

namespace knowledge_extraction::kleene::position {
class AtTrafficSignExtractor : public KleeneExtractor {
  private:
    const TrafficSignTypes traffic_sign_type;

  public:
    AtTrafficSignExtractor(std::shared_ptr<knowledge_extraction::env_model::EnvironmentModel> env_model,
                           Proposition proposition, TrafficSignTypes traffic_sign_type)
        : KleeneExtractor(std::move(env_model), proposition), traffic_sign_type(traffic_sign_type) {}

    std::unordered_map<time_step_t, TrueFalseObstacleIds>
    extract(const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>>
                &relevant_obstacle_ids_over_time) const override;
};
} // namespace knowledge_extraction::kleene::position
