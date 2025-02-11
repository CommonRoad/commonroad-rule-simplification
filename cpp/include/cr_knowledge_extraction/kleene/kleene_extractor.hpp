#pragma once

#include "cr_knowledge_extraction/env_model/env_model.hpp"
#include "cr_knowledge_extraction/proposition.hpp"

#include <commonroad_cpp/auxiliaryDefs/types_and_definitions.h>

#include <memory>
#include <unordered_set>
#include <utility>

namespace knowledge_extraction::kleene {
class KleeneExtractor {
  private:
    const Proposition proposition;

  protected:
    const std::shared_ptr<knowledge_extraction::env_model::EnvironmentModel> env_model;

  public:
    /**
     * Create an extractor for Kleene knowledge.
     *
     * These extractors will decide whether a predicate must be true or cannot be true and return unknown otherwise.
     *
     * @param env_model The environment model.
     * @param proposition The proposition corresponding to the predicate handled by this extractor.
     */
    KleeneExtractor(std::shared_ptr<knowledge_extraction::env_model::EnvironmentModel> env_model,
                    Proposition proposition)
        : proposition(proposition), env_model(std::move(env_model)){};

    virtual ~KleeneExtractor() = default;

    /**
     * Get the associated proposition.
     *
     * @return The proposition.
     */
    Proposition get_proposition() const { return proposition; }

    /**
     * Return value for extraction.
     *
     * Consists of two sets of obstacle IDs.
     * The first set indicates the obstacles for which the predicate must be true, the second set contains the obstacles
     * for which the predicate must be false. The ID std::nullopt indicates the ego vehicle.
     */
    using TrueFalseObstacleIds =
        std::pair<std::unordered_set<std::optional<size_t>>, std::unordered_set<std::optional<size_t>>>;

    /**
     * Extract Kleene knowledge.
     *
     * @param relevant_obstacle_ids_over_time Map of time steps to relevant obstacle IDs, like above std::nullopt
     *     indicates the ego vehicle.
     * @return The extracted knowledge for each time step.
     */
    virtual std::unordered_map<time_step_t, TrueFalseObstacleIds>
    extract(const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>>
                &relevant_obstacle_ids_over_time) const = 0;
};
} // namespace knowledge_extraction::kleene
