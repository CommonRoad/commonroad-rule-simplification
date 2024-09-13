#pragma once

#include "cr_knowledge_extraction/ego_behavior/behavior_overapproximation.hpp"
#include "cr_knowledge_extraction/kleene/ego_dependent_kleene_extractor.hpp"

namespace knowledge_extraction::kleene::position {
class OnMainCarriagewayRightLaneExtractor : public EgoDependentKleeneExtractor {
  private:
    static bool is_mcw_right_lane_lanelet(const std::shared_ptr<Lanelet> &lanelet);

  public:
    OnMainCarriagewayRightLaneExtractor(
        std::shared_ptr<World> world, std::shared_ptr<geometry::CurvilinearCoordinateSystem> ego_ccs,
        std::shared_ptr<knowledge_extraction::ego_behavior::BehaviorOverapproximation> approximations)
        : EgoDependentKleeneExtractor(std::move(world), std::move(ego_ccs), Proposition::ON_MAIN_CARRIAGEWAY_RIGHT_LANE,
                                      std::move(approximations)) {}

    std::unordered_map<time_step_t, TrueFalseObstacleIds>
    extract(const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>>
                &relevant_obstacle_ids_over_time) const override;
};
} // namespace knowledge_extraction::kleene::position
