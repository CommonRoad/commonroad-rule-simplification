#include "cr_knowledge_extraction/road_network/curvilinear_road_network.hpp"

#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/road_network.h>

#include <ranges>

using namespace knowledge_extraction::road_network;

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using RTreeValue = std::pair<box, size_t>;
struct CurvilinearRoadNetwork::RTree {
    bgi::rtree<RTreeValue, bgi::quadratic<16>> rtree;
};

knowledge_extraction::road_network::CurvilinearRoadNetwork::CurvilinearRoadNetwork(
    const std::shared_ptr<World> &world, const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs)
    : lanelets(initialize(world, ego_ccs)), rtree(std::make_shared<RTree>()) {
    // Populate boost rtree with bounding boxes of curvilinear lanelets
    for (const auto &pair : this->lanelets) {
        box bounding_box;
        bg::envelope(pair.second->curvilinear_polygon, bounding_box);
        rtree->rtree.insert(std::make_pair(bounding_box, pair.first));
    }
}

polygon_type CurvilinearRoadNetwork::convert_to_ccs(const polygon_type &polygon,
                                                    const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs) {
    geometry::EigenPolyline outer_polyline;
    for (const auto &vertex : polygon.outer()) {
        if (ccs->cartesianPointInProjectionDomain(vertex.x(), vertex.y())) {
            outer_polyline.emplace_back(vertex.x(), vertex.y());
        }
    }
    auto ccs_polyline = ccs->convertListOfPointsToCurvilinearCoords(outer_polyline, 4);
    polygon_type ccs_polygon;
    ccs_polygon.outer().reserve(ccs_polyline.size());
    for (const auto &point : ccs_polyline) {
        ccs_polygon.outer().emplace_back(point.x(), point.y());
    }
    return ccs_polygon;
}

std::unordered_map<size_t, std::shared_ptr<CurvilinearLanelet>>
CurvilinearRoadNetwork::initialize(const std::shared_ptr<World> &world,
                                   const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs) {
    // Remove lanelets that are not convertible to the ego CCS
    auto convertible_lanelets =
        world->getRoadNetwork()->getLaneletNetwork() | std::views::filter([&](const auto &lanelet) {
            // check if any vertex is in the projection domain of the ego CCS
            const auto &vertices = lanelet->getOuterPolygon().outer();
            return std::any_of(vertices.begin(), vertices.end(), [&](const auto &vertex) {
                return ego_ccs->cartesianPointInProjectionDomain(vertex.x(), vertex.y());
            });
        });
    // Convert to ego CCS
    auto curvilinear_lanelets =
        convertible_lanelets | std::views::transform([&](const auto &lanelet) {
            return std::make_pair(lanelet->getId(), std::make_shared<CurvilinearLanelet>(
                                                        lanelet, convert_to_ccs(lanelet->getOuterPolygon(), ego_ccs)));
        });
    return {curvilinear_lanelets.begin(), curvilinear_lanelets.end()};
}

std::shared_ptr<CurvilinearLanelet>
knowledge_extraction::road_network::CurvilinearRoadNetwork::get_lanelet(size_t lanelet_id) const {
    if (lanelets.contains(lanelet_id)) {
        return lanelets.at(lanelet_id);
    } else {
        throw(std::invalid_argument("Lanelet with ID " + std::to_string(lanelet_id) + " not found."));
    }
}

std::vector<std::shared_ptr<CurvilinearLanelet>>
knowledge_extraction::road_network::CurvilinearRoadNetwork::get_overlapping_lanelets(
    const knowledge_extraction::ego_behavior::sets::Box2D &bounding_box) const {
    // convert to boost geometry box
    auto [min, max] = bounding_box.bounds();
    auto bg_box = box{{min(0), min(1)}, {max(0), max(1)}};
    // find all relevant lanelets by making use of the rtree
    std::vector<RTreeValue> relevant_lanelets;
    rtree->rtree.query(bgi::intersects(bg_box), std::back_inserter(relevant_lanelets));

    auto overlapping_lanelets =
        relevant_lanelets |
        std::views::transform([this](const auto &rtree_val) { return get_lanelet(rtree_val.second); }) |
        std::views::filter(
            [&bg_box](const auto &lanelet) { return bg::intersects(lanelet->curvilinear_polygon, bg_box); });

    return {overlapping_lanelets.begin(), overlapping_lanelets.end()};
}
