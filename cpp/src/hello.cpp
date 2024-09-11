#include "cr_knowledge_extraction/hello.hpp"
#include "cr_knowledge_extraction/kleene/kleene_extractor.hpp"
#include "cr_knowledge_extraction/proposition.hpp"
#include "cr_knowledge_extraction/relationship/relationship_extractor.hpp"

// CCS
#include <geometry/curvilinear_coordinate_system.h>

// Environment model
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>

std::string knowledge_extraction::hello() {
    geometry::CurvilinearCoordinateSystem ccs{{{0, 0}, {1, 1}, {2, 2}}};
    auto roadNetwork = std::make_shared<RoadNetwork>(std::vector<std::shared_ptr<Lanelet>>{});
    World world{"TestWorld", 0, roadNetwork, {}, {}, 0.1};
    return "Hello, CommonRoad (from C++)!";
}
