#pragma once

#include "cr_knowledge_extraction/ego_behavior/behavior_overapproximation.hpp"
#include "cr_knowledge_extraction/kleene/ego_dependent_kleene_extractor.hpp"

namespace knowledge_extraction::kleene::position {
class InFrontOfExtractor : public EgoDependentKleeneExtractor {
  private:
    std::optional<double> get_obstacle_rear(size_t time_step, const std::shared_ptr<Obstacle> &obstacle) const;

  public:
    InFrontOfExtractor(std::shared_ptr<World> world, std::shared_ptr<geometry::CurvilinearCoordinateSystem> ego_ccs,
                       std::shared_ptr<knowledge_extraction::ego_behavior::BehaviorOverapproximation> approximations)
        : EgoDependentKleeneExtractor(std::move(world), std::move(ego_ccs), Proposition::IN_FRONT_OF,
                                      std::move(approximations)) {}

    std::unordered_map<time_step_t, TrueFalseObstacleIds>
    extract(const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>>
                &relevant_obstacle_ids_over_time) const override;
};
} // namespace knowledge_extraction::kleene::position
