#pragma once

#include "cr_knowledge_extraction/ego_behavior/behavior_overapproximation.hpp"
#include "cr_knowledge_extraction/kleene/kleene_extractor.hpp"

namespace knowledge_extraction::kleene {

class EgoDependentKleeneExtractor : public knowledge_extraction::kleene::KleeneExtractor {
  protected:
    const std::shared_ptr<knowledge_extraction::ego_behavior::BehaviorOverapproximation> approximations;

  public:
    EgoDependentKleeneExtractor(
        std::shared_ptr<World> world, std::shared_ptr<geometry::CurvilinearCoordinateSystem> ego_ccs,
        Proposition proposition,
        std::shared_ptr<knowledge_extraction::ego_behavior::BehaviorOverapproximation> approximations)
        : KleeneExtractor(std::move(world), std::move(ego_ccs), proposition),
          approximations(std::move(approximations)) {}
};

} // namespace knowledge_extraction::kleene
