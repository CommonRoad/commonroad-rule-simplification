#pragma once

#include "cr_knowledge_extraction/env_model/env_model.hpp"
#include "cr_knowledge_extraction/proposition.hpp"

#include <commonroad_cpp/auxiliaryDefs/types_and_definitions.h>

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
    const std::shared_ptr<knowledge_extraction::env_model::EnvironmentModel> env_model;

  public:
    /**
     * Create an extractor for relationships between two propositions.
     *
     * Relationships can be implications or equivalences.
     *
     * @param env_model The environment model.
     * @param proposition_lhs The proposition corresponding to the left-hand side predicate.
     * @param proposition_rhs The proposition corresponding to the right-hand side predicate.
     * @param dominant_relationship The more common relationship type between the two propositions.
     */
    RelationshipExtractor(std::shared_ptr<knowledge_extraction::env_model::EnvironmentModel> env_model,
                          Proposition proposition_lhs, Proposition proposition_rhs,
                          RelationshipType dominant_relationship)
        : proposition_lhs(proposition_lhs), proposition_rhs(proposition_rhs),
          dominant_relationship(dominant_relationship), env_model(std::move(env_model)){};

    virtual ~RelationshipExtractor() = default;

    /**
     * Get the associated propositions.
     *
     * @return The propositions.
     */
    std::pair<Proposition, Proposition> get_propositions() const { return {proposition_lhs, proposition_rhs}; }

    /**
     * Get the relationship type that is most commonly extracted by this extractor.
     *
     * @return The dominant relationship type.
     */
    RelationshipType get_dominant_relationship() const { return dominant_relationship; }

    /**
     * Return value for extraction.
     *
     * Consists of the type of relationship (implication or equivalence) and the IDs of the obstacles that are related.
     */
    using Relationship = std::tuple<RelationshipType, size_t, size_t>;

    /**
     * Extract relationships between the two propositions.
     *
     * @param relevant_obstacle_ids_over_time Map of time steps to relevant obstacle IDs, std::nullopt
     *     indicates the ego vehicle.
     * @return The extracted relationships for each time step.
     */
    virtual std::unordered_map<time_step_t, std::vector<Relationship>>
    extract(const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>>
                &relevant_obstacle_ids_over_time) const = 0;
};
} // namespace knowledge_extraction::relationship
