#pragma once

#include "cr_knowledge_extraction/ego_behavior/ego_params.hpp"
#include "cr_knowledge_extraction/ego_behavior/sets/box.hpp"
#include "cr_knowledge_extraction/road_network/curvilinear_road_network.hpp"

#include <Eigen/Dense>
#include <commonroad_cpp/auxiliaryDefs/types_and_definitions.h>

namespace knowledge_extraction::ego_behavior {
class BehaviorOverapproximation {
  private:
    const road_network::CurvilinearRoadNetwork ccs_road_network;

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
    /**
     * Construct a behavior overapproximation for the ego vehicle.
     *
     * This allows us to conservatively bound the future behavior of position and velocity of the ego vehicle.
     *
     * @param dt The time step size in s.
     * @param ego_params The configuration parameters of the ego vehicle.
     * @param ccs_road_network The curvilinear road network.
     */
    BehaviorOverapproximation(double dt, const EgoParameters &ego_params,
                              road_network::CurvilinearRoadNetwork ccs_road_network);

    /**
     * Get the radius of inscribed circle of the ego vehicle shape.
     *
     * @return The radius of the inscribed circle.
     */
    double get_inner_radius() const { return shrink_delta; }

    /**
     * Get the radius of circumscribed circle of the ego vehicle shape.
     *
     * @return The radius of the circumscribed circle.
     */
    double get_outer_radius() const { return outer_shape_box.radius(0); }

    /**
     * Get the minimal longitudinal position of the ego vehicle at the given time step.
     *
     * @param time_step The time step.
     * @return The minimal longitudinal position in m.
     */
    double p_lon_min(time_step_t time_step) {
        const auto &[min, _] = get_center_approximation(time_step).bounds();
        return min(0);
    }

    /**
     * Get the maximal longitudinal position of the ego vehicle at the given time step.
     *
     * @param time_step The time step.
     * @return The maximal longitudinal position in m.
     */
    double p_lon_max(time_step_t time_step) {
        const auto &[_, max] = get_center_approximation(time_step).bounds();
        return max(0);
    }

    /**
     * Get the minimal lateral position of the ego vehicle at the given time step.
     *
     * @param time_step The time step.
     * @return The minimal lateral position in m.
     */
    double p_lat_min(time_step_t time_step) {
        const auto &[min, _] = get_center_approximation(time_step).bounds();
        return min(2);
    }

    /**
     * Get the maximal lateral position of the ego vehicle at the given time step.
     *
     * @param time_step The time step.
     * @return The maximal lateral position in m.
     */
    double p_lat_max(time_step_t time_step) {
        const auto &[_, max] = get_center_approximation(time_step).bounds();
        return max(2);
    }

    /**
     * Get the minimal longitudinal velocity of the ego vehicle at the given time step.
     *
     * @param time_step The time step.
     * @return The minimal longitudinal velocity in $\frac{m}{s}$.
     */
    double v_lon_min(time_step_t time_step) {
        const auto &[min, _] = get_center_approximation(time_step).bounds();
        return min(1);
    }

    /**
     * Get the maximal longitudinal velocity of the ego vehicle at the given time step.
     *
     * @param time_step The time step.
     * @return The maximal longitudinal velocity in $\frac{m}{s}$.
     */
    double v_lon_max(time_step_t time_step) {
        const auto &[_, max] = get_center_approximation(time_step).bounds();
        return max(1);
    }

    /**
     * Get the minimal lateral velocity of the ego vehicle at the given time step.
     *
     * @param time_step The time step.
     * @return The minimal lateral velocity in $\frac{m}{s}$.
     */
    double v_lat_min(time_step_t time_step) {
        const auto &[min, _] = get_center_approximation(time_step).bounds();
        return min(3);
    }

    /**
     * Get the maximal lateral velocity of the ego vehicle at the given time step.
     *
     * @param time_step The time step.
     * @return The maximal lateral velocity in $\frac{m}{s}$.
     */
    double v_lat_max(time_step_t time_step) {
        const auto &[_, max] = get_center_approximation(time_step).bounds();
        return max(3);
    }

    /**
     * Get the minimal absolute velocity of the ego vehicle at the given time step.
     *
     * @param time_step The time step.
     * @return The minimal absolute velocity in $\frac{m}{s}$.
     */
    double v_min(time_step_t time_step) { return get_velocity_approximation(time_step).first; }

    /**
     * Get the maximal absolute velocity of the ego vehicle at the given time step.
     *
     * @param time_step The time step.
     * @return The maximal absolute velocity in $\frac{m}{s}$.
     */
    double v_max(time_step_t time_step) { return get_velocity_approximation(time_step).second; }

    /**
     * Get a box approximating the state of the ego vehicle at the given time step.
     *
     * The center of the ego vehicle is used as the reference point.
     * The state of the ego vehicle is given as $(s, \dot{s}, d, \dot{d})$, where $s$ indicates the longitudinal
     * direction and $d$ the lateral direction.
     *
     * @param time_step The time step.
     * @return The box approximating the four-dimensional state of the ego vehicle.
     */
    sets::Box4D get_center_approximation(time_step_t time_step);

    /**
     * Get a box containing all positions $(s, d)$ that the ego vehicle may occupy at the given time step.
     *
     * @param time_step The time step.
     * @return The box overapproximating the occupancy of the ego vehicle.
     */
    sets::Box2D get_occupancy_approximation(time_step_t time_step);

    /**
     * Get the lanelets that the ego vehicle might occupy at the given time step.
     *
     * @param time_step The time step.
     * @return The potentially covered lanelets.
     */
    const std::vector<std::shared_ptr<Lanelet>> &get_covered_lanelets(time_step_t time_step);

    /**
     * Get a box in the position domain $(s, d)$ so that the ego vehicle will surely intersect with at least one point
     * in this box at the given time step.
     *
     * @param time_step The time step.
     * @return The box descibed above.
     */
    sets::Box2D get_occupancy_intersection_approximation(time_step_t time_step);

    /**
     * Get a set of lanelets so that the occupancy of the ego vehicle at the given time step surely intersects with at
     * least one of them.
     *
     * @param time_step The time step.
     * @return The set of intersected lanelets.
     */
    const std::vector<std::shared_ptr<Lanelet>> &get_intersected_lanelets(time_step_t time_step);

    /**
     * Get the minimum and maximum absolute velocity of the ego vehicle possible at the given time step.
     *
     * @param time_step The time step.
     * @return A pair of minimum and maximum absolute velocity.
     */
    const std::pair<double, double> &get_velocity_approximation(time_step_t time_step);

    /**
     * Get the minimum and maximal priority for the given turning direction of the ego vehicle possible at the given
     * time step.
     *
     * @param time_step The time step.
     * @param dir The turning direction.
     * @return A pair of minimum and maximum priority.
     */
    const std::pair<int, int> &get_priority_range(time_step_t time_step, TurningDirection dir);
};
} // namespace knowledge_extraction::ego_behavior
