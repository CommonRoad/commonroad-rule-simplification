#pragma once

#include <commonroad_cpp/obstacle/state.h>

namespace knowledge_extraction::ego_behavior {
struct EgoParameters {
    double a_lon_min{-11.5};
    double a_lon_max{11.5};
    double a_lat_min{-2.0};
    double a_lat_max{2.0};

    double v_lon_min{-13.9};
    double v_lon_max{50.8};
    double v_lat_min{-4.0};
    double v_lat_max{4.0};

    State initial_state{0, 0, 0, 0, 0, 0};
    double uncertainty_p_lon{0.01};
    double uncertainty_p_lat{0.01};
    double uncertainty_v_lon{0.01};
    double uncertainty_v_lat{0.01};

    double length{4.5};
    double width{1.8};

    double t_react{0.3};
};
} // namespace knowledge_extraction::ego_behavior
