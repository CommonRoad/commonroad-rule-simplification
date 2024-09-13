#pragma once

#include "cr_knowledge_extraction/ego_behavior/sets/box.hpp"
#include "cr_knowledge_extraction/road_network/curvilinear_lanelet.hpp"

#include <commonroad_cpp/world.h>
#include <geometry/curvilinear_coordinate_system.h>

namespace knowledge_extraction::road_network {
class CurvilinearRoadNetwork {
  private:
    const std::unordered_map<size_t, std::shared_ptr<CurvilinearLanelet>> lanelets;

    static polygon_type convert_to_ccs(const polygon_type &polygon,
                                       const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs);

    static std::unordered_map<size_t, std::shared_ptr<CurvilinearLanelet>>
    initialize(const std::shared_ptr<World> &lanelet,
               const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs);

    struct RTree;
    const std::shared_ptr<RTree> rtree;

  public:
    CurvilinearRoadNetwork(const std::shared_ptr<World> &world,
                           const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs);

    std::shared_ptr<CurvilinearLanelet> get_lanelet(size_t lanelet_id) const;

    std::vector<std::shared_ptr<CurvilinearLanelet>>
    get_overlapping_lanelets(const knowledge_extraction::ego_behavior::sets::Box2D &bounding_box) const;
};
} // namespace knowledge_extraction::road_network
