#pragma once

#include "cr_knowledge_extraction/ego_behavior/behavior_overapproximation.hpp"
#include "cr_knowledge_extraction/kleene/ego_dependent_kleene_extractor.hpp"

namespace knowledge_extraction::kleene::position {
class OnLaneletWithTypeExtractor : public EgoDependentKleeneExtractor {
  private:
    const LaneletType lanelet_type;

  public:
    OnLaneletWithTypeExtractor(
        std::shared_ptr<World> world, std::shared_ptr<geometry::CurvilinearCoordinateSystem> ego_ccs,
        Proposition proposition,
        std::shared_ptr<knowledge_extraction::ego_behavior::BehaviorOverapproximation> approximations,
        LaneletType lanelet_type)
        : EgoDependentKleeneExtractor(std::move(world), std::move(ego_ccs), proposition, std::move(approximations)),
          lanelet_type(lanelet_type) {}

    std::unordered_map<time_step_t, TrueFalseObstacleIds>
    extract(const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>>
                &relevant_obstacle_ids_over_time) const override;
};
} // namespace knowledge_extraction::kleene::position
