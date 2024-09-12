#include "cr_knowledge_extraction/ego_behavior/behavior_overapproximation.hpp"

#include <numbers>
#include <utility>

using namespace knowledge_extraction::ego_behavior;
using knowledge_extraction::ego_behavior::sets::Box2D;
using knowledge_extraction::ego_behavior::sets::Box4D;

BehaviorOverapproximation::BehaviorOverapproximation(double dt, const EgoParameters &ego_params)
    : system_matrix(make_system_matrix(dt)), input_state_update(make_input_state_update(dt, ego_params)),
      admissible_states(make_admissible_states(ego_params)),
      shrink_delta(compute_shrink_delta(ego_params.length, ego_params.width)),
      outer_shape_box(make_outer_shape_box(ego_params.length, ego_params.width)),
      offset(ego_params.initial_state.getTimeStep()),
      center_approximation({make_initial_center_approximation(ego_params)}) {}

Eigen::Matrix<double, 4, 4> BehaviorOverapproximation::make_system_matrix(double dt) {
    return Eigen::Matrix4d{
        {1, dt, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, dt},
        {0, 0, 0, 1},
    };
}

Box4D BehaviorOverapproximation::make_input_state_update(double dt, const EgoParameters &ego_params) {
    auto half_dt_square = (dt * dt) / 2;
    auto input_matrix = Eigen::Matrix<double, 4, 2>{
        {half_dt_square, 0},
        {dt, 0},
        {0, half_dt_square},
        {0, dt},
    };
    auto input_set = Box2D::from_bounds(Eigen::Vector2d{ego_params.a_lon_min, ego_params.a_lat_min},
                                        Eigen::Vector2d{ego_params.a_lon_max, ego_params.a_lat_max});
    return input_set.linear_map_positive(input_matrix);
}

sets::Box4D BehaviorOverapproximation::make_admissible_states(const EgoParameters &ego_params) {
    return Box4D::from_bounds(Eigen::Vector4d{std::numeric_limits<double>::min(), ego_params.v_lon_min,
                                              std::numeric_limits<double>::min(), ego_params.v_lat_min},
                              Eigen::Vector4d{std::numeric_limits<double>::max(), ego_params.v_lon_max,
                                              std::numeric_limits<double>::max(), ego_params.v_lat_max});
}

double BehaviorOverapproximation::compute_shrink_delta(double length, double width) {
    auto inner_radius = length > width ? width / 2 : length / 2;
    return std::numbers::sqrt2 * inner_radius;
}

sets::Box2D BehaviorOverapproximation::make_outer_shape_box(double length, double width) {
    auto length_half = length / 2;
    auto width_half = width / 2;
    auto side_half = std::sqrt((length_half * length_half) + (width_half * width_half));
    return Box2D{{0, 0}, {side_half, side_half}};
}

sets::Box4D BehaviorOverapproximation::make_initial_center_approximation(const EgoParameters &ego_params) {
    const auto &initial_state = ego_params.initial_state;

    auto theta = initial_state.getCurvilinearOrientation();
    auto velocity = initial_state.getVelocity();
    auto lon_velocity = velocity * std::cos(theta);
    auto lat_velocity = velocity * std::sin(theta);

    auto lon_pos = initial_state.getLonPosition();
    auto lat_pos = initial_state.getLatPosition();

    auto center = Eigen::Vector4d{lon_pos, lon_velocity, lat_pos, lat_velocity};
    auto radius = Eigen::Vector4d{ego_params.uncertainty_p_lon, ego_params.uncertainty_v_lon,
                                  ego_params.uncertainty_p_lat, ego_params.uncertainty_v_lat};
    return Box4D{center, radius};
}

sets::Box4D BehaviorOverapproximation::get_center_approximation(time_step_t time_step) {
    auto idx = time_step - offset;
    if (center_approximation.size() <= idx) {
        // compute missing steps
        auto previous = get_center_approximation(time_step - 1);
        auto next = previous.linear_map_positive(system_matrix).sum(input_state_update).intersect(admissible_states);
        center_approximation.emplace_back(std::move(next));
    }
    return center_approximation[idx];
}

sets::Box2D BehaviorOverapproximation::get_occupancy_approximation(time_step_t time_step) {
    auto idx = time_step - offset;
    if (occupancy_approximation.size() <= idx) {
        // compute all previous steps
        get_occupancy_approximation(time_step - 1);
        auto center_approx = get_center_approximation(time_step);
        auto occ_approx = project_to_positions(center_approx).sum(outer_shape_box);
        occupancy_approximation.emplace_back(std::move(occ_approx));
    }
    return occupancy_approximation[idx];
}

sets::Box2D BehaviorOverapproximation::get_occupancy_intersection_approximation(time_step_t time_step) {
    auto idx = time_step - offset;
    if (occupancy_intersection_approximation.size() <= idx) {
        // compute all previous steps
        get_occupancy_intersection_approximation(time_step - 1);
        auto center_approx = get_center_approximation(time_step);
        auto occ_int_approx = project_to_positions(center_approx).shrink(shrink_delta);
        occupancy_intersection_approximation.emplace_back(std::move(occ_int_approx));
    }
    return occupancy_intersection_approximation[idx];
}

sets::Box2D BehaviorOverapproximation::project_to_positions(const Box4D &state_set) {
    auto center = state_set.center({0, 2});
    auto radius = state_set.radius({0, 2});
    return Box2D{center, radius};
}
