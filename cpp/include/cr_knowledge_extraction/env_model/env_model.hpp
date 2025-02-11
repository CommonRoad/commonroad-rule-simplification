#pragma once

#include "cr_knowledge_extraction/ego_behavior/behavior_overapproximation.hpp"
#include "cr_knowledge_extraction/ego_behavior/ego_params.hpp"

#include <boost/functional/hash.hpp>
#include <commonroad_cpp/predicates/predicate_parameter_collection.h>
#include <commonroad_cpp/world.h>
#include <geometry/curvilinear_coordinate_system.h>
#include <utility>

namespace knowledge_extraction::env_model {
class EnvironmentModel {
  private:
    const std::shared_ptr<World> world;
    const std::shared_ptr<geometry::CurvilinearCoordinateSystem> ego_ccs;
    const ego_behavior::EgoParameters ego_params;
    PredicateParameters predicate_params;

    const std::shared_ptr<ego_behavior::BehaviorOverapproximation> ego_approximations;
    static std::shared_ptr<ego_behavior::BehaviorOverapproximation>
    make_ego_approximations(const std::shared_ptr<World> &world,
                            const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs,
                            ego_behavior::EgoParameters ego_params);

    template <typename T> using ObstacleCache =
        std::unordered_map<std::pair<time_step_t, size_t>, T, boost::hash<std::pair<time_step_t, size_t>>>;

    ObstacleCache<std::optional<double>> obstacle_rear_cache;
    std::optional<double> get_obstacle_rear_impl(size_t time_step, const std::shared_ptr<Obstacle> &obstacle) const;

    ObstacleCache<std::optional<std::set<size_t>>> obstacle_lane_ids_cache;
    std::optional<std::set<size_t>> get_obstacle_lane_ids_impl(size_t time_step,
                                                               const std::shared_ptr<Obstacle> &obstacle) const;

    ObstacleCache<std::optional<double>> stopping_s_cache;
    std::optional<double> get_stopping_s_impl(size_t time_step, const std::shared_ptr<Obstacle> &obstacle);

    std::unordered_map<size_t, std::unordered_set<TurningDirection>> turning_directions_cache;
    std::unordered_set<TurningDirection> get_turning_directions_impl(const std::shared_ptr<Obstacle> &obstacle);

    std::unordered_map<std::tuple<time_step_t, size_t, TurningDirection>, std::optional<int>,
                       boost::hash<std::tuple<time_step_t, size_t, TurningDirection>>>
        priority_cache;

  public:
    /**
     * Create a wrapper around the given world to cache results that are needed often.
     *
     * @param world The C++ world object corresponding to the CommonRoad scenario.
     * @param ego_ccs The curvilinear coordinate system of the ego vehicle.
     * @param ego_params The configuration parameters of the ego vehicle.
     * @param predicate_params The traffic rule predicate parameters.
     */
    EnvironmentModel(std::shared_ptr<World> world, std::shared_ptr<geometry::CurvilinearCoordinateSystem> ego_ccs,
                     const ego_behavior::EgoParameters &ego_params, PredicateParameters predicate_params)
        : world(std::move(world)), ego_ccs(std::move(ego_ccs)), ego_params(ego_params),
          predicate_params(std::move(predicate_params)),
          ego_approximations(make_ego_approximations(this->world, this->ego_ccs, this->ego_params)) {}

    /**
     * Get the behavior approximation of the ego vehicle.
     *
     * @return The behavior approximation.
     */
    const std::shared_ptr<ego_behavior::BehaviorOverapproximation> &get_ego_approximations() const {
        return ego_approximations;
    }

    /**
     * Get the curvilinear coordinate system of the ego vehicle.
     *
     * @return The curvilinear coordinate system.
     */
    const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &get_ego_ccs() const { return ego_ccs; }

    /**
     * Get the C++ world object corresponding to the CommonRoad scenario
     *
     * @return The world object.
     */
    const std::shared_ptr<World> &get_world() const { return world; }

    /**
     * Get the configuration parameters of the ego vehicle.
     *
     * @return The ego vehicle parameters.
     */
    const ego_behavior::EgoParameters &get_ego_params() const { return ego_params; }

    /**
     * Get the traffic rule predicate parameters.
     *
     * @return The predicate parameters.
     */
    PredicateParameters &get_predicate_params() { return predicate_params; }

    /**
     * Get the rear-most s-coordinate of the given obstacle in the CCS of the ego vehicle.
     *
     * @param time_step The time step of interest
     * @param obstacle The obstacle.
     * @return The rear-most s-coordinate at the given time step or std::nullopt if the coordinate conversion is not
     *     possible.
     */
    std::optional<double> get_obstacle_rear(size_t time_step, const std::shared_ptr<Obstacle> &obstacle);

    /**
     * Get the IDs of the lanelets that the obstacle occupies at the given time step.
     *
     * @param time_step The time step.
     * @param obstacle The obstacle.
     * @return The set of occupied lanelet IDs or std::nullopt if there was an error getting the lanelets.
     */
    std::optional<std::set<size_t>> get_obstacle_lane_ids(size_t time_step, const std::shared_ptr<Obstacle> &obstacle);

    /**
     * Get the rear s-coordinate at which the obstacle would stop if it were to fully brake.
     *
     * Uses the CCS of the ego vehicle.
     *
     * @param time_step The time step of interest.
     * @param obstacle The obstacle.
     * @return The s-coordinate of std::nullopt if the coordinate conversion is not possible.
     */
    std::optional<double> get_stopping_s(size_t time_step, const std::shared_ptr<Obstacle> &obstacle);

    /**
     * Get the possible turning directions of an obstacle.
     *
     * Note that this is time step independent.
     *
     * @param obstacle The obstacle.
     * @return The possible turning directions.
     */
    const std::unordered_set<TurningDirection> &get_turning_directions(const std::shared_ptr<Obstacle> &obstacle);

    /**
     * Get the priority of the obstacle for the given turning direction.
     *
     * @param time_step The time step of interest.
     * @param obstacle The obstacle.
     * @param dir The turning direction.
     * @return The priority or std::nullopt if there was an error when determining the priorities.
     */
    std::optional<int> get_priority(size_t time_step, const std::shared_ptr<Obstacle> &obstacle, TurningDirection dir);
};
} // namespace knowledge_extraction::env_model
