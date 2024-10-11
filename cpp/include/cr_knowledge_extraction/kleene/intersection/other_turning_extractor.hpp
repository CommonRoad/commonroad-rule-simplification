#pragma once

#include "cr_knowledge_extraction/kleene/kleene_extractor.hpp"

namespace knowledge_extraction::kleene::intersection {
class OtherTurningExtractor : public KleeneExtractor {
  private:
    const TurningDirection direction;

  public:
    OtherTurningExtractor(std::shared_ptr<knowledge_extraction::env_model::EnvironmentModel> env_model,
                          Proposition prop, TurningDirection direction)
        : KleeneExtractor(std::move(env_model), prop), direction(direction) {}

    std::unordered_map<time_step_t, TrueFalseObstacleIds>
    extract(const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>> &opt) const override;
};
} // namespace knowledge_extraction::kleene::intersection
