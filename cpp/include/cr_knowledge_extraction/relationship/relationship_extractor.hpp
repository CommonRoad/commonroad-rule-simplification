#pragma once

#include "cr_knowledge_extraction/proposition.hpp"

#include <commonroad_cpp/auxiliaryDefs/types_and_definitions.h>
#include <commonroad_cpp/world.h>
#include <geometry/curvilinear_coordinate_system.h>

#include <memory>
#include <unordered_set>
#include <utility>

namespace knowledge_extraction::relationship {
enum class RelationshipType : std::uint8_t { IMPLICATION, EQUIVALENCE };

class RelationshipExtractor {
  private:
    const Proposition proposition_lhs;
    const Proposition proposition_rhs;
    const RelationshipType dominant_relationship;

  protected:
    const std::shared_ptr<World> world;
    const std::shared_ptr<geometry::CurvilinearCoordinateSystem> ego_ccs;

  public:
    RelationshipExtractor(std::shared_ptr<World> world, std::shared_ptr<geometry::CurvilinearCoordinateSystem> ego_ccs,
                          Proposition proposition_lhs, Proposition proposition_rhs,
                          RelationshipType dominant_relationship)
        : proposition_lhs(proposition_lhs), proposition_rhs(proposition_rhs),
          dominant_relationship(dominant_relationship), world(std::move(world)), ego_ccs(std::move(ego_ccs)){};

    virtual ~RelationshipExtractor() = default;

    std::pair<Proposition, Proposition> get_propositions() const { return {proposition_lhs, proposition_rhs}; }

    RelationshipType get_dominant_relationship() const { return dominant_relationship; }

    using Relationship = std::tuple<RelationshipType, size_t, size_t>;

    virtual std::unordered_map<time_step_t, std::vector<Relationship>>
    extract(const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>>
                &relevant_obstacle_ids_over_time) const = 0;
};
} // namespace knowledge_extraction::relationship
