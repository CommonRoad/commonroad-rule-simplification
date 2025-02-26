#pragma once

#include "cr_knowledge_extraction/env_model/env_model.hpp"
#include "cr_knowledge_extraction/kleene/kleene_extractor.hpp"
#include "cr_knowledge_extraction/relationship/relationship_extractor.hpp"

#include <commonroad_cpp/auxiliaryDefs/types_and_definitions.h>
#include <commonroad_cpp/world.h>
#include <geometry/curvilinear_coordinate_system.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace knowledge_extraction {
/**
 * The result of knowledge extraction at a specific time step.
 */
struct ExtractionResult {
    std::vector<std::string> positive_propositions;
    std::vector<std::string> negative_propositions;
    std::vector<std::pair<std::string, std::string>> implications;
    std::vector<std::pair<std::string, std::string>> equivalences;
};

class ExtractionInterface {
  private:
    std::shared_ptr<env_model::EnvironmentModel> env_model;
    time_step_t initial_time_step;

    std::optional<std::unique_ptr<kleene::KleeneExtractor>> create_kleene_extractor(Proposition prop);

    std::optional<std::unique_ptr<relationship::RelationshipExtractor>> create_relationship_extractor(Proposition prop);

    // We use std::nullopt to mark the ego vehicle
    using RelevantObstacles =
        std::unordered_map<Proposition, std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>>>;

    /**
     * Determine the relevant obstacles for each proposition over time.
     *
     * @param relevant_propositions The propositions that are relevant for extraction at each time step.
     * @return A map from propositions to relevant obstacle IDs over time. We use std::nullopt to mark the ego vehicle.
     */
    RelevantObstacles compute_relevant_obstacles(
        const std::unordered_map<time_step_t, std::vector<std::string>> &relevant_propositions) const;

    /**
     * Extract Kleene knowledge for the relevant obstacles.
     *
     * @param relevant_obstacles The relevant obstacles for each proposition over time.
     * @param result Output parameter for the extraction results.
     */
    void extract_kleene(const RelevantObstacles &relevant_obstacles,
                        std::unordered_map<time_step_t, ExtractionResult> &result);

    /**
     * Extract relationships for the relevant obstacles.
     *
     * @param relevant_obstacles The relevant obstacles for each proposition over time.
     * @param result Output parameter for the extraction results.
     * @param type If given, extract mostly relationships of this type.
     */
    void extract_relationships(const RelevantObstacles &relevant_obstacles,
                               std::unordered_map<time_step_t, ExtractionResult> &result,
                               std::optional<relationship::RelationshipType> type = std::nullopt);

  public:
    /**
     * Create an interface for knowledge extraction.
     *
     * @param world The C++ world object corresponding to the CommonRoad scenario.
     * @param ego_ccs The curvilinear coordinate system of the ego vehicle.
     * @param ego_params The configuration parameters of the ego vehicle.
     */
    ExtractionInterface(std::shared_ptr<World> world, std::shared_ptr<geometry::CurvilinearCoordinateSystem> ego_ccs,
                        const ego_behavior::EgoParameters &ego_params)
        : env_model(std::make_shared<env_model::EnvironmentModel>(std::move(world), std::move(ego_ccs), ego_params,
                                                                  PredicateParameters{})),
          initial_time_step(ego_params.initial_state.getTimeStep()) {}
    // TODO: Make predicate parameters configurable from Python

    /**
     * Extract all knowledge for the relevant propositions.
     *
     * @param relevant_propositions The propositions that are relevant for extraction at each time step.
     * @return The extracted knowledge for each time step.
     */
    std::unordered_map<time_step_t, ExtractionResult>
    extract_all(const std::unordered_map<time_step_t, std::vector<std::string>> &relevant_propositions);

    /**
     * Extract all knowledge except implications for the relevant propositions.
     *
     * @param relevant_propositions The propositions that are relevant for extraction at each time step.
     * @return The extracted knowledge for each time step.
     */
    std::unordered_map<time_step_t, ExtractionResult> extract_all_but_implications(
        const std::unordered_map<time_step_t, std::vector<std::string>> &relevant_propositions);

    /**
     * Extract only Kleene knowledge for the relevant propositions.
     *
     * @param relevant_propositions The propositions that are relevant for extraction at each time step.
     * @return The extracted knowledge for each time step.
     */
    std::unordered_map<time_step_t, ExtractionResult>
    extract_kleene(const std::unordered_map<time_step_t, std::vector<std::string>> &relevant_propositions);

    /**
     * Extract only relationships for the relevant propositions.
     *
     * @param relevant_propositions The propositions that are relevant for extraction at each time step.
     * @return The extracted knowledge for each time step.
     */
    std::unordered_map<time_step_t, ExtractionResult>
    extract_relationships(const std::unordered_map<time_step_t, std::vector<std::string>> &relevant_propositions);

    /**
     * Extract only equivalences for the relevant propositions.
     *
     * @param relevant_propositions The propositions that are relevant for extraction at each time step.
     * @return The extracted knowledge for each time step.
     */
    std::unordered_map<time_step_t, ExtractionResult>
    extract_equivalences(const std::unordered_map<time_step_t, std::vector<std::string>> &relevant_propositions);

    /**
     * Extract only implications for the relevant propositions.
     *
     * @param relevant_propositions The propositions that are relevant for extraction at each time step.
     * @return The extracted knowledge for each time step.
     */
    std::unordered_map<time_step_t, ExtractionResult>
    extract_implications(const std::unordered_map<time_step_t, std::vector<std::string>> &relevant_propositions);
};
} // namespace knowledge_extraction
