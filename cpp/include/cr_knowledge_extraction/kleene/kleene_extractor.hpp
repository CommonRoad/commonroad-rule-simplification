#pragma once

#include "cr_knowledge_extraction/proposition.hpp"

#include <commonroad_cpp/auxiliaryDefs/types_and_definitions.h>
#include <commonroad_cpp/world.h>
#include <geometry/curvilinear_coordinate_system.h>

#include <memory>
#include <unordered_set>
#include <utility>

namespace knowledge_extraction::kleene {
class KleeneExtractor {
  private:
    const Proposition proposition;

  protected:
    const std::shared_ptr<World> world;
    const std::shared_ptr<geometry::CurvilinearCoordinateSystem> ego_ccs;

  public:
    KleeneExtractor(std::shared_ptr<World> world, std::shared_ptr<geometry::CurvilinearCoordinateSystem> ego_ccs,
                    Proposition proposition)
        : proposition(proposition), world(std::move(world)), ego_ccs(std::move(ego_ccs)){};

    virtual ~KleeneExtractor() = default;

    Proposition get_proposition() const { return proposition; }

    using TrueFalseObstacleIds =
        std::pair<std::unordered_set<std::optional<size_t>>, std::unordered_set<std::optional<size_t>>>;

    virtual std::unordered_map<time_step_t, TrueFalseObstacleIds>
    extract(const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>>
                &relevant_obstacle_ids_over_time) const = 0;
};
} // namespace knowledge_extraction::kleene
