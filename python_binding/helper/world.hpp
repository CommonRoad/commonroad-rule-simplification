#pragma once

#include <commonroad_cpp/interfaces/commonroad/input_utils.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/world.h>

namespace pybind_helper {

/**
 * Resample the states of a trajectory with a given step width and adjusts their time stamps.
 *
 * @tparam T Type of state
 * @param current_trajectory Trajectory to resample.
 * @param step_width Only states at time steps that are multiples of this value are kept.
 * @return Filtered trajectory with adjusted time stamps.
 */
template <typename T> Obstacle::time_step_map_t<T>
resample_trajectory(const Obstacle::time_step_map_t<T> &current_trajectory, size_t step_width);

/**
 * Resample the states of the obstacles with a given step width and adjusts their time stamps.
 *
 * @param obstacles Obstacles to resample.
 * @param step_width Only states at time steps that are multiples of this value are kept.
 */
void resample_obstacle_states(const std::vector<std::shared_ptr<Obstacle>> &obstacles, size_t step_width);

/**
 * Resample the traffic light cycles of a road network with a given step width.
 *
 * @param road_network The road network to resample.
 * @param step_width Duration and offset of each traffic light cycle are divided by this value.
 */
void resample_road_network(const std::shared_ptr<RoadNetwork> &road_network, size_t step_width);

/**
 * Load a world object from a CommonRoad scenario file and resample the scenario to a new time step.
 *
 * @param scenario_path Path to the CommonRoad scenario file.
 * @param new_dt New time step size (must be a multiple of the original time step size).
 * @return A world object representing the resampled scenario.
 */
std::shared_ptr<World> open_world(const std::string &scenario_path, double new_dt);

template <typename T> Obstacle::time_step_map_t<T>
resample_trajectory(const Obstacle::time_step_map_t<T> &current_trajectory, size_t step_width) {
    Obstacle::time_step_map_t<T> resampled_trajectory{};
    for (const auto &[time_step, state] : current_trajectory) {
        if (time_step % step_width == 0) {
            state->setTimeStep(time_step / step_width);
            resampled_trajectory[time_step / step_width] = state;
        }
    }
    return resampled_trajectory;
}

} // namespace pybind_helper
