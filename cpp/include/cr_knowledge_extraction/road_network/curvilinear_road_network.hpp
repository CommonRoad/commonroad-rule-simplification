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
    /**
     * Construct a road network that is described in the curvilinear coordinates of the ego vehicle.
     *
     * @param road_network The road network in Cartesian coordinates.
     * @param ego_ccs The curvilinear coordinate system of the ego vehicle.
     */
    CurvilinearRoadNetwork(const std::shared_ptr<RoadNetwork> &road_network,
                           const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs);

    /**
     * Find all lanelets that overlap with the given bounding box.
     *
     * @param ccs_bounding_box The bounding box in curvilinear coordinates.
     * @return The lanelets that overlap with the bounding box.
     */
    std::vector<std::shared_ptr<Lanelet>>
    get_overlapping_lanelets(const knowledge_extraction::ego_behavior::sets::Box2D &ccs_bounding_box) const;
};
} // namespace knowledge_extraction::road_network
