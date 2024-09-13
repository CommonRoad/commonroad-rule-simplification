#pragma once

#include "cr_knowledge_extraction/ego_behavior/behavior_overapproximation.hpp"
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
struct ExtractionResult {
    std::vector<std::string> positive_propositions;
    std::vector<std::string> negative_propositions;
    std::vector<std::pair<std::string, std::string>> implications;
    std::vector<std::pair<std::string, std::string>> equivalences;
};

class ExtractionInterface {
  private:
    std::shared_ptr<World> world;
    std::shared_ptr<geometry::CurvilinearCoordinateSystem> ego_ccs;

    std::shared_ptr<ego_behavior::BehaviorOverapproximation> ego_approximations;
    static std::shared_ptr<ego_behavior::BehaviorOverapproximation>
    make_ego_approximations(const std::shared_ptr<World> &world,
                            const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs,
                            ego_behavior::EgoParameters ego_params);

    std::optional<std::unique_ptr<kleene::KleeneExtractor>> create_kleene_extractor(Proposition prop);

    std::optional<std::unique_ptr<relationship::RelationshipExtractor>> create_relationship_extractor(Proposition prop);

    using RelevantObstacles =
        std::unordered_map<Proposition, std::unordered_map<time_step_t, std::unordered_set<size_t>>>;

    static RelevantObstacles
    compute_relevant_obstacles(const std::unordered_map<time_step_t, std::vector<std::string>> &relevant_propositions);

    void extract_kleene(const RelevantObstacles &relevant_obstacles,
                        std::unordered_map<time_step_t, ExtractionResult> &result);

    void extract_relationships(const ExtractionInterface::RelevantObstacles &relevant_obstacles,
                               std::unordered_map<time_step_t, ExtractionResult> &result,
                               std::optional<relationship::RelationshipType> type = std::nullopt);

  public:
    ExtractionInterface(std::shared_ptr<World> world, std::shared_ptr<geometry::CurvilinearCoordinateSystem> ego_ccs,
                        const ego_behavior::EgoParameters &ego_params)
        : world(std::move(world)), ego_ccs(std::move(ego_ccs)),
          ego_approximations(make_ego_approximations(this->world, this->ego_ccs, ego_params)) {}

    std::unordered_map<time_step_t, ExtractionResult>
    extract_all(const std::unordered_map<time_step_t, std::vector<std::string>> &relevant_propositions);

    std::unordered_map<time_step_t, ExtractionResult>
    extract_kleene(const std::unordered_map<time_step_t, std::vector<std::string>> &relevant_propositions);

    std::unordered_map<time_step_t, ExtractionResult>
    extract_relationships(const std::unordered_map<time_step_t, std::vector<std::string>> &relevant_propositions);

    std::unordered_map<time_step_t, ExtractionResult>
    extract_equivalences(const std::unordered_map<time_step_t, std::vector<std::string>> &relevant_propositions);

    std::unordered_map<time_step_t, ExtractionResult>
    extract_implications(const std::unordered_map<time_step_t, std::vector<std::string>> &relevant_propositions);
};
} // namespace knowledge_extraction
