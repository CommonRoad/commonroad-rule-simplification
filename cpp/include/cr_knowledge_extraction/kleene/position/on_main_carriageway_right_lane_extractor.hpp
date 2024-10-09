#pragma once

#include "cr_knowledge_extraction/kleene/kleene_extractor.hpp"

namespace knowledge_extraction::kleene::position {
class OnMainCarriagewayRightLaneExtractor : public KleeneExtractor {
  private:
    static bool is_mcw(const std::shared_ptr<Lanelet> &lanelet);

    static bool is_rightmost(const std::shared_ptr<Lanelet> &lanelet);

    static bool is_neighbour_opposite(const std::shared_ptr<Lanelet> &lanelet);

  public:
    OnMainCarriagewayRightLaneExtractor(std::shared_ptr<knowledge_extraction::env_model::EnvironmentModel> env_model)
        : KleeneExtractor(std::move(env_model), Proposition::ON_MAIN_CARRIAGEWAY_RIGHT_LANE) {}

    std::unordered_map<time_step_t, TrueFalseObstacleIds>
    extract(const std::unordered_map<time_step_t, std::unordered_set<std::optional<size_t>>>
                &relevant_obstacle_ids_over_time) const override;
};
} // namespace knowledge_extraction::kleene::position
