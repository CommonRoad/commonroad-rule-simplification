#include "cr_knowledge_extraction/road_network/curvilinear_road_network.hpp"

using namespace knowledge_extraction::road_network;

knowledge_extraction::road_network::CurvilinearRoadNetwork::CurvilinearRoadNetwork(
    const std::shared_ptr<RoadNetwork> &road_network,
    const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs)
    : road_network(road_network), ego_ccs(ego_ccs) {}

std::vector<std::shared_ptr<Lanelet>>
knowledge_extraction::road_network::CurvilinearRoadNetwork::get_overlapping_lanelets(
    const knowledge_extraction::ego_behavior::sets::Box2D &ccs_bounding_box) const {
    auto [min, max] = ccs_bounding_box.bounds();

    try {
        // Convert the bounding box back to the Cartesian coordinate system
        [[maybe_unused]] std::vector<geometry::EigenPolyline> _triangle_mesh;
        auto cart_polygon = ego_ccs->convertRectangleToCartesianCoords(min(0), max(0), min(1), max(1), _triangle_mesh);

        // Convert the drivability checker polygon into a boost::geometry polygon
        polygon_type bg_polygon;
        bg_polygon.outer().reserve(cart_polygon.size());
        for (const auto &point : cart_polygon) {
            bg_polygon.outer().emplace_back(point.x(), point.y());
        }

        // Query the Cartesian road network using the polygon
        multi_polygon_type bg_multi_polygon;
        bg_multi_polygon.push_back(bg_polygon);
        return road_network->findOccupiedLaneletsByShape(bg_multi_polygon);
    } catch (const geometry::CurvilinearProjectionDomainError &e) {
        // If we cannot convert from the CCS, be conservative and return all lanelets
        return road_network->getLaneletNetwork();
    }
}
