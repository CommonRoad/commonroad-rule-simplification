#pragma once

#include <commonroad_cpp/interfaces/commonroad/input_utils.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/world.h>

namespace pybind_helper {

/**
 * Load a world object from a CommonRoad scenario file and resample the scenario to a new time step.
 *
 * @param scenario_path Path to the CommonRoad scenario file.
 * @param new_dt New time step size (must be a multiple of the original time step size).
 * @return A world object representing the resampled scenario.
 */
std::shared_ptr<World> open_world(const std::string &scenario_path);

} // namespace pybind_helper
