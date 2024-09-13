#pragma once

#include <utility>

#include "cr_knowledge_extraction/relationship/relationship_extractor.hpp"

namespace knowledge_extraction::relationship::equivalence {
class InSameLaneEquivExtractor : public RelationshipExtractor {
  private:
    std::optional<std::set<size_t>> get_obstacle_lane_ids(size_t time_step,
                                                          const std::shared_ptr<Obstacle> &obstacle) const;

  public:
    InSameLaneEquivExtractor(std::shared_ptr<World> world,
                             std::shared_ptr<geometry::CurvilinearCoordinateSystem> ego_ccs)
        : RelationshipExtractor(std::move(world), std::move(ego_ccs), Proposition::IN_SAME_LANE,
                                Proposition::IN_SAME_LANE, RelationshipType::EQUIVALENCE){};

    std::unordered_map<time_step_t, std::vector<Relationship>>
    extract(const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>>
                &relevant_obstacle_ids_over_time) const override;
};
} // namespace knowledge_extraction::relationship::equivalence
