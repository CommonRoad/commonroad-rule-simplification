#include "world.hpp"

#include <commonroad_cpp/roadNetwork/road_network.h>

void pybind_helper::resample_obstacle_states(const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                             size_t step_width) {
    for (const auto &obs : obstacles) {
        auto current_state = obs->getCurrentState();
        auto current_signal_state = obs->getCurrentSignalState();
        auto prediction = obs->getTrajectoryPrediction();
        auto signal_series = obs->getSignalSeries();
        if (current_state && current_state->getTimeStep() % step_width != 0) {
            auto current_time_step = current_state->getTimeStep();
            auto mod = current_time_step % step_width;
            // Take the first state of the trajectory prediction that matches the step width as current state
            current_state = nullptr;
            current_signal_state = nullptr;
            for (auto timeStep = current_time_step + step_width - mod; timeStep <= obs->getFinalTimeStep();
                 timeStep += step_width) {
                if (prediction.contains(timeStep)) {
                    current_state = prediction.at(timeStep);
                    // We also set the current signal state if it exists
                    if (signal_series.contains(timeStep)) {
                        current_signal_state = signal_series.at(timeStep);
                        signal_series.erase(timeStep);
                    }
                    prediction.erase(timeStep);
                    break;
                }
            }
        }
        if (current_state) {
            current_state->setTimeStep(current_state->getTimeStep() / step_width);
        }
        if (current_signal_state) {
            current_signal_state->setTimeStep(current_signal_state->getTimeStep() / step_width);
        }
        obs->setCurrentState(current_state);
        obs->setCurrentSignalState(current_signal_state);
        obs->setTrajectoryPrediction(pybind_helper::resample_trajectory(prediction, step_width));
        obs->setTrajectoryHistory(pybind_helper::resample_trajectory(obs->getTrajectoryHistory(), step_width));
        // TODO: Enable this once the setters are available
        // obs->setSignalSeries(pybind_helper::resample_trajectory(signal_series, step_width));
        // obs->setSignalSeriesHistory(pybind_helper::resample_trajectory(obs->getSignalSeriesHistory(), step_width));
    }
}

void pybind_helper::resample_road_network(const std::shared_ptr<RoadNetwork> &road_network, size_t step_width) {
    for (const auto &traffic_light : road_network->getTrafficLights()) {
        traffic_light->setOffset(traffic_light->getOffset() / step_width);
        for (auto &cycle_element : traffic_light->getCycle()) {
            cycle_element.duration /= step_width;
        }
    }
}

std::shared_ptr<World> pybind_helper::open_world(const std::string &scenario_path, double new_dt) {
    // Create environment model
    const auto &[obstacles, roadNetwork, scenario_dt] = InputUtils::getDataFromCommonRoad(scenario_path);
    if (new_dt != scenario_dt) {
        constexpr int numeric_scaling = 100;
        if (fmod((new_dt * numeric_scaling), (scenario_dt * numeric_scaling)) != 0) {
            throw std::logic_error("New dt is not a multiple of scenario dt");
        }
        auto step_width = static_cast<size_t>(round((new_dt * numeric_scaling) / (scenario_dt * numeric_scaling)));
        pybind_helper::resample_obstacle_states(obstacles, step_width);
        pybind_helper::resample_road_network(roadNetwork, step_width);
    }
    // World always starts at time 0, and we don't have any ego vehicles in the world
    return std::make_shared<World>("knowledgeExtractionWorld", 0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{},
                                   obstacles, new_dt);
}
