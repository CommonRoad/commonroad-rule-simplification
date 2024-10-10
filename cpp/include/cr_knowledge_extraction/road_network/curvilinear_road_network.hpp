#pragma once

#include "cr_knowledge_extraction/ego_behavior/sets/box.hpp"
#include "cr_knowledge_extraction/road_network/curvilinear_lanelet.hpp"

#include <commonroad_cpp/roadNetwork/road_network.h>
#include <geometry/curvilinear_coordinate_system.h>

namespace knowledge_extraction::road_network {
class CurvilinearRoadNetwork {
  private:
    const std::shared_ptr<RoadNetwork> road_network;
    const std::shared_ptr<geometry::CurvilinearCoordinateSystem> ego_ccs;

  public:
    CurvilinearRoadNetwork(const std::shared_ptr<RoadNetwork> &road_network,
                           const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs);

    std::vector<std::shared_ptr<Lanelet>>
    get_overlapping_lanelets(const knowledge_extraction::ego_behavior::sets::Box2D &ccs_bounding_box) const;
};
} // namespace knowledge_extraction::road_network
