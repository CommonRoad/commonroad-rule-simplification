#pragma once

#include "cr_knowledge_extraction/env_model/env_model.hpp"
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
    const std::shared_ptr<knowledge_extraction::env_model::EnvironmentModel> env_model;

  public:
    KleeneExtractor(std::shared_ptr<knowledge_extraction::env_model::EnvironmentModel> env_model,
                    Proposition proposition)
        : proposition(proposition), env_model(std::move(env_model)){};

    virtual ~KleeneExtractor() = default;

    Proposition get_proposition() const { return proposition; }

    using TrueFalseObstacleIds =
        std::pair<std::unordered_set<std::optional<size_t>>, std::unordered_set<std::optional<size_t>>>;

    virtual std::unordered_map<time_step_t, TrueFalseObstacleIds>
    extract(const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>>
                &relevant_obstacle_ids_over_time) const = 0;
};
} // namespace knowledge_extraction::kleene
