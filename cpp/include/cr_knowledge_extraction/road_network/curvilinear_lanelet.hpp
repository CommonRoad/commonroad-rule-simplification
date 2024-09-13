#pragma once

#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>

namespace knowledge_extraction::road_network {
struct CurvilinearLanelet {
    const std::shared_ptr<Lanelet> lanelet;
    const polygon_type curvilinear_polygon;
};
} // namespace knowledge_extraction::road_network
