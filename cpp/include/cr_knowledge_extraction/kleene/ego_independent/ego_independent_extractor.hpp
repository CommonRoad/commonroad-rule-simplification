#pragma once

#include "cr_knowledge_extraction/kleene/kleene_extractor.hpp"

#include <commonroad_cpp/predicates/commonroad_predicate.h>

namespace knowledge_extraction::kleene::ego_independent {
class EgoIndependentExtractor : public KleeneExtractor {
  private:
    const std::unique_ptr<CommonRoadPredicate> inner_predicate;
    const std::vector<std::string> additional_params;

    /**
     * Evaluate the inner predicate at the given step in the given world for the given obstacle.
     *
     * @param step The current time step.
     * @param world The world object representing the scenario.
     * @param obstacle_id The ID of the relevant obstacle.
     * @return True iff the inner predicate is satisfied.
     * @throw std::logic_error If the obstacle does not exist in the world.
     */
    bool evaluate_inner(time_step_t step, size_t obstacle_id) const;

  public:
    EgoIndependentExtractor(std::shared_ptr<World> world,
                            std::shared_ptr<geometry::CurvilinearCoordinateSystem> ego_ccs, Proposition proposition,
                            std::unique_ptr<CommonRoadPredicate> inner_predicate,
                            std::vector<std::string> additional_params)
        : KleeneExtractor(std::move(world), std::move(ego_ccs), proposition),
          inner_predicate(std::move(inner_predicate)), additional_params(std::move(additional_params)) {}

    std::unordered_map<time_step_t, TrueFalseObstacleIds>
    extract(const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>>
                &relevant_obstacle_ids_over_time) const override;
};
} // namespace knowledge_extraction::kleene::ego_independent
