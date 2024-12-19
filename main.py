import copy
import time
from typing import Tuple

import crcpp
import more_itertools
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.util import Interval
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import InitialState
from commonroad.scenario.trajectory import Trajectory
from commonroad_dc import pycrccosy
from commonroad_route_planner.route_planner import RoutePlanner
from ltl_augmentation import Formula

from cr_rule_simplification import EgoParameters
from cr_rule_simplification.instantiation.traffic_rule_instantiator import TrafficRuleInstantiator
from cr_rule_simplification.simplification.traffic_rule_simplifier import TrafficRuleSimplifier


def main():
    # scenario_path = "scenarios/DEU_LocationALower-11_4_T-1.xml"
    # scenario_path = "cpp/tests/scenarios/interstate_simple.xml"
    scenario_path = "scenarios/DEU_MerzenichRather-2_8814400_T-14549.xml"
    scenario, planning_problems = CommonRoadFileReader(scenario_path).open()
    dt = 0.16
    scenario, planning_problems = resample_scenario(scenario, planning_problems, dt)
    planning_problem = list(planning_problems.planning_problem_dict.values())[0]
    planning_horizon = 15
    world = crcpp.World(scenario)

    # configure ego parameters
    ego_params = EgoParameters(initial_state=initialize_from_planning_problem(planning_problem))

    # plan route and create clcs
    route = RoutePlanner(scenario, planning_problem).plan_routes().retrieve_first_route()
    reference_path = pycrccosy.Util.resample_polyline(route.reference_path, 2.0)
    ccs = pycrccosy.CurvilinearCoordinateSystem(reference_path)

    # instantiate traffic rules
    tic = time.perf_counter()
    rules = list(
        more_itertools.flatten(
            TrafficRuleInstantiator()
            .instantiate(["R_I5"], scenario, planning_problem, time_steps=planning_horizon)
            .values()
        )
    )
    toc = time.perf_counter()
    print(Formula.conjunction(rules))
    print(f"Instantiation took {toc - tic:0.4f} seconds")

    # simplify traffic rules
    tic = time.perf_counter()
    simplifier = TrafficRuleSimplifier(world, ego_params, ccs)
    simplified_rules = simplifier.simplify(rules, planning_horizon)
    toc = time.perf_counter()
    print(Formula.conjunction(simplified_rules))
    print(f"Simplification took {toc - tic:0.4f} seconds")


def initialize_from_planning_problem(
    planning_problem: PlanningProblem,
) -> Tuple[int, float, float, float, float, float]:
    state = planning_problem.initial_state
    # there is no acceleration in the planning problem, so we set it to 0
    return state.time_step, state.position[0], state.position[1], state.velocity, 0, state.orientation


def resample_scenario(
    scenario: Scenario, planning_problems: PlanningProblemSet, new_dt: float
) -> Tuple[Scenario, PlanningProblemSet]:
    """Resamples a scenario to a new time step size (only works for scenarios with trajectory predictions)."""
    assert new_dt > 0, "New time step size must be positive"
    assert scenario.dt <= new_dt, "New time step size must be larger than the old one"
    assert new_dt % scenario.dt == 0, "New time step size must be a multiple of the old one"

    scenario_downsampled = copy.deepcopy(scenario)
    pps_downsampled = copy.deepcopy(planning_problems)

    step_width = int(round(new_dt / scenario.dt))

    if step_width == 1:
        return scenario_downsampled, pps_downsampled

    for planning_problem in pps_downsampled.planning_problem_dict.values():
        planning_problem.initial_state.time_step = planning_problem.initial_state.time_step // step_width
        planning_problem.goal.state_list = _resample_state_list(planning_problem.goal.state_list, step_width)

    scenario_downsampled.dt = new_dt
    for obs in scenario_downsampled.dynamic_obstacles:
        prediction_state_list = obs.prediction.trajectory.state_list
        if obs.initial_state.time_step % step_width != 0:
            # find minimum time step in prediction that is a multiple of step_width
            step = min(
                state.time_step
                for state in prediction_state_list
                if state.time_step % step_width == 0 and state.time_step >= obs.initial_state.time_step
            )
            initial_state = obs.prediction.trajectory.state_at_time_step(step)
            prediction_state_list.remove(initial_state)
            obs.initial_state = initial_state.convert_state_to_state(InitialState())

            obs.initial_signal_state = obs.signal_state_at_time_step(step)
            if obs.initial_signal_state:
                obs.signal_series.remove(obs.initial_signal_state)

            # Unset initial lanelet info
            obs.initial_center_lanelet_ids = None
            obs.initial_shape_lanelet_ids = None
        obs.initial_state.time_step //= step_width
        if obs.initial_signal_state:
            obs.initial_signal_state.time_step //= step_width
        resampled_prediction_state_list = _resample_state_list(prediction_state_list, step_width)
        if resampled_prediction_state_list:
            obs.prediction.trajectory = Trajectory(
                resampled_prediction_state_list[0].time_step, resampled_prediction_state_list
            )
        else:
            obs.prediction = None
        obs.history = _resample_state_list(obs.history, step_width)
        obs.signal_series = _resample_state_list(obs.signal_series, step_width)
        obs.signal_history = _resample_state_list(obs.signal_history, step_width)
        # unset meta information states
        # TODO: Implement once this is needed
        obs.initial_meta_information_state = None
        obs.meta_information_series = None

    for traffic_light in scenario_downsampled.lanelet_network.traffic_lights:
        traffic_light.traffic_light_cycle.time_offset //= step_width
        for cycle_element in traffic_light.traffic_light_cycle.cycle_elements:
            cycle_element.duration //= step_width

    return scenario_downsampled, pps_downsampled


def _resample_state_list(state_list: list, step_width: int) -> list:
    """Resamples a list of states with the given step width."""
    state_list = [
        state
        for state in state_list
        if (isinstance(state.time_step, int) and state.time_step % step_width == 0)
        or (isinstance(state.time_step, Interval) and state.time_step.length >= step_width)
    ]
    for state in state_list:
        if isinstance(state.time_step, Interval):
            state.time_step = Interval(state.time_step.start // step_width, state.time_step.end // step_width)
        else:
            state.time_step //= step_width
    return state_list


if __name__ == "__main__":
    main()
