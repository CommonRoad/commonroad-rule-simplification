#pragma once

#include "cr_knowledge_extraction/kleene/kleene_extractor.hpp"

namespace knowledge_extraction::kleene::regulatory {
class PriorityExtractor : public KleeneExtractor {
  public:
    enum class PriorityMode : std::uint8_t { EGO_HAS_PRIORITY, OTHER_HAS_PRIORITY, SAME_PRIORITY };

  private:
    const TurningDirection ego_turn;
    const TurningDirection other_turn;

    const PriorityMode mode;

    static std::optional<bool> ego_has_prio(int ego_prio_min, int ego_prio_max, int obs_prio);
    static std::optional<bool> other_has_prio(int ego_prio_min, int ego_prio_max, int obs_prio);
    static std::optional<bool> same_prio(int ego_prio_min, int ego_prio_max, int obs_prio);

  public:
    PriorityExtractor(std::shared_ptr<knowledge_extraction::env_model::EnvironmentModel> env_model, Proposition prop,
                      PriorityMode mode, TurningDirection ego_turn, TurningDirection other_turn)
        : KleeneExtractor(std::move(env_model), prop), ego_turn(ego_turn), other_turn(other_turn), mode(mode) {}

    std::unordered_map<time_step_t, TrueFalseObstacleIds>
    extract(const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>>
                &relevant_obstacle_ids_over_time) const override;
};
} // namespace knowledge_extraction::kleene::regulatory
