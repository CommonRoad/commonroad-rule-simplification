#pragma once

#include "cr_knowledge_extraction/ego_behavior/ego_params.hpp"
#include "cr_knowledge_extraction/ego_behavior/sets/box.hpp"
#include "cr_knowledge_extraction/road_network/curvilinear_road_network.hpp"

#include <Eigen/Dense>
#include <commonroad_cpp/auxiliaryDefs/types_and_definitions.h>

namespace knowledge_extraction::ego_behavior {
class BehaviorOverapproximation {
  private:
    const knowledge_extraction::road_network::CurvilinearRoadNetwork ccs_road_network;

    const Eigen::Matrix<double, 4, 4> system_matrix;
    static Eigen::Matrix<double, 4, 4> make_system_matrix(double dt);

    const sets::Box4D input_state_update;
    static sets::Box4D make_input_state_update(double dt, const EgoParameters &ego_params);

    const sets::Box4D admissible_states;
    static sets::Box4D make_admissible_states(const EgoParameters &ego_params);

    const double shrink_delta;
    static double compute_shrink_delta(double length, double width);

    const sets::Box2D outer_shape_box;
    static sets::Box2D make_outer_shape_box(double length, double width);

    const time_step_t offset;

    std::vector<sets::Box4D> center_approximation;
    static sets::Box4D make_initial_center_approximation(const EgoParameters &ego_params);

    std::unordered_map<time_step_t, sets::Box2D> occupancy_approximation;
    std::unordered_map<time_step_t, std::vector<std::shared_ptr<Lanelet>>> covered_lanelets;
    std::unordered_map<std::pair<time_step_t, TurningDirection>, std::pair<int, int>,
                       boost::hash<std::pair<time_step_t, TurningDirection>>>
        priority_range;

    std::unordered_map<time_step_t, sets::Box2D> occupancy_intersection_approximation;
    std::unordered_map<time_step_t, std::vector<std::shared_ptr<Lanelet>>> intersected_lanelets;

    std::unordered_map<time_step_t, std::pair<double, double>> velocity_approximation;

    static sets::Box2D project_to_positions(const sets::Box4D &state_set);

  public:
    BehaviorOverapproximation(double dt, const EgoParameters &ego_params,
                              knowledge_extraction::road_network::CurvilinearRoadNetwork ccs_road_network);

    double get_inner_radius() const { return shrink_delta; }

    double get_outer_radius() const { return outer_shape_box.radius(0); }

    double p_lon_min(time_step_t time_step) {
        const auto &[min, _] = get_center_approximation(time_step).bounds();
        return min(0);
    }

    double p_lon_max(time_step_t time_step) {
        const auto &[_, max] = get_center_approximation(time_step).bounds();
        return max(0);
    }

    double p_lat_min(time_step_t time_step) {
        const auto &[min, _] = get_center_approximation(time_step).bounds();
        return min(2);
    }

    double p_lat_max(time_step_t time_step) {
        const auto &[_, max] = get_center_approximation(time_step).bounds();
        return max(2);
    }

    double v_lon_min(time_step_t time_step) {
        const auto &[min, _] = get_center_approximation(time_step).bounds();
        return min(1);
    }

    double v_lon_max(time_step_t time_step) {
        const auto &[_, max] = get_center_approximation(time_step).bounds();
        return max(1);
    }

    double v_lat_min(time_step_t time_step) {
        const auto &[min, _] = get_center_approximation(time_step).bounds();
        return min(3);
    }

    double v_lat_max(time_step_t time_step) {
        const auto &[_, max] = get_center_approximation(time_step).bounds();
        return max(3);
    }

    double v_min(time_step_t time_step) { return get_velocity_approximation(time_step).first; }

    double v_max(time_step_t time_step) { return get_velocity_approximation(time_step).second; }

    sets::Box4D get_center_approximation(time_step_t time_step);

    sets::Box2D get_occupancy_approximation(time_step_t time_step);
    const std::vector<std::shared_ptr<Lanelet>> &get_covered_lanelets(time_step_t time_step);

    sets::Box2D get_occupancy_intersection_approximation(time_step_t time_step);
    const std::vector<std::shared_ptr<Lanelet>> &get_intersected_lanelets(time_step_t time_step);

    const std::pair<double, double> &get_velocity_approximation(time_step_t time_step);

    const std::pair<int, int> &get_priority_range(time_step_t time_step, TurningDirection dir);
};
} // namespace knowledge_extraction::ego_behavior
