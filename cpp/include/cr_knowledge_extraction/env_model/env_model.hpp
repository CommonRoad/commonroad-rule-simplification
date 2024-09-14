#pragma once

#include "cr_knowledge_extraction/ego_behavior/behavior_overapproximation.hpp"
#include "cr_knowledge_extraction/ego_behavior/ego_params.hpp"

#include <boost/functional/hash.hpp>
#include <commonroad_cpp/world.h>
#include <geometry/curvilinear_coordinate_system.h>

namespace knowledge_extraction::env_model {
class EnvironmentModel {
  private:
    const std::shared_ptr<World> world;
    const std::shared_ptr<geometry::CurvilinearCoordinateSystem> ego_ccs;
    const knowledge_extraction::ego_behavior::EgoParameters ego_params;

    const std::shared_ptr<knowledge_extraction::ego_behavior::BehaviorOverapproximation> ego_approximations;
    static std::shared_ptr<knowledge_extraction::ego_behavior::BehaviorOverapproximation>
    make_ego_approximations(const std::shared_ptr<World> &world,
                            const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ego_ccs,
                            knowledge_extraction::ego_behavior::EgoParameters ego_params);

    template <typename T> using ObstacleCache =
        std::unordered_map<std::pair<time_step_t, size_t>, T, boost::hash<std::pair<time_step_t, size_t>>>;

    ObstacleCache<std::optional<double>> obstacle_rear_cache;
    std::optional<double> get_obstacle_rear_impl(size_t time_step, const std::shared_ptr<Obstacle> &obstacle) const;

    ObstacleCache<std::optional<std::set<size_t>>> obstacle_lane_ids_cache;
    std::optional<std::set<size_t>> get_obstacle_lane_ids_impl(size_t time_step,
                                                               const std::shared_ptr<Obstacle> &obstacle) const;

  public:
    EnvironmentModel(std::shared_ptr<World> world, std::shared_ptr<geometry::CurvilinearCoordinateSystem> ego_ccs,
                     const knowledge_extraction::ego_behavior::EgoParameters &ego_params)
        : world(std::move(world)), ego_ccs(std::move(ego_ccs)), ego_params(ego_params),
          ego_approximations(make_ego_approximations(this->world, this->ego_ccs, this->ego_params)) {}

    const std::shared_ptr<knowledge_extraction::ego_behavior::BehaviorOverapproximation> &
    get_ego_approximations() const {
        return ego_approximations;
    }

    const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &get_ego_ccs() const { return ego_ccs; }

    const std::shared_ptr<World> &get_world() const { return world; }

    const knowledge_extraction::ego_behavior::EgoParameters &get_ego_params() const { return ego_params; }

    std::optional<double> get_obstacle_rear(size_t time_step, const std::shared_ptr<Obstacle> &obstacle);
    std::optional<std::set<size_t>> get_obstacle_lane_ids(size_t time_step, const std::shared_ptr<Obstacle> &obstacle);
};
} // namespace knowledge_extraction::env_model
