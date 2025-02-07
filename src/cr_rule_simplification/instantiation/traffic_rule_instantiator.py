import math
import warnings
from typing import Callable, Dict, List, Optional, Set

import numpy as np
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.obstacle import Obstacle
from commonroad.scenario.scenario import Scenario
from ltl_augmentation import Formula

from cr_rule_simplification.instantiation.R_G1 import SafeDistanceRule
from cr_rule_simplification.instantiation.R_I5 import EnteringVehiclesRule
from cr_rule_simplification.instantiation.R_IN1 import StopSignRule
from cr_rule_simplification.instantiation.R_IN3 import RightBeforeLeftRule
from cr_rule_simplification.instantiation.R_IN4 import PriorityRule
from cr_rule_simplification.instantiation.R_IN5 import TurningLeftRule


class TrafficRuleInstantiator:
    """A class to manage the instantiation of traffic rules.

    Supported rules:
        - R_G1 (SafeDistanceRule)
        - R_I5 (EnteringVehiclesRule)
        - R-IN1 (StopSignRule)
        - R-IN3 (RightBeforeLeftRule)
        - R-IN4 (PriorityRule)
        - R-IN5 (TurningLeftRule)
    """

    fov_radius: float

    t_c: float
    t_slw: float
    t_ia: float
    t_ib: float

    def __init__(
        self, fov_radius: float = 200, t_c: float = 3.0, t_slw: float = 3.0, t_ia: float = 0.5, t_ib: float = 1.0
    ):
        """Create a new TrafficRuleInstantiator.

        :param fov_radius: The field of view radius in which obstacles are considered.
        :param t_c: The $t_c$ parameter for the safe distance rule in s.
        :param t_slw: The $t_{slw}$ parameter for the stop sign rule in s.
        :param t_ia: The $t_{ia}$ parameter for the "not_endanger_intersection" meta-predicate in s.
        :param t_ib: The $t_{ib}$ parameter for the "not_endanger_intersection" meta-predicate in s.
        """
        self.fov_radius = fov_radius

        self.t_c = t_c
        self.t_slw = t_slw
        self.t_ia = t_ia
        self.t_ib = t_ib

    def instantiate(
        self,
        rules: List[str],
        scenario: Scenario,
        planning_problem: PlanningProblem,
        planning_horizon: Optional[int] = None,
        consider_obstacle: Callable[[Obstacle], bool] = lambda obs: True,
    ) -> Dict[str, List[Formula]]:
        """Instantiate a list of traffic rules for a given scenario and planning problem.

        :param rules: The list of rule names.
        :param scenario: The CommonRoad scenario.
        :param planning_problem: The planning problem.
        :param planning_horizon: The planning horizon for which to instantiate the rules.
            If None, the rules have to hold indefinitely.
        :param consider_obstacle: A function that returns True for each obstacle that should be considered in the
            instantiation.
        :return: A dictionary mapping each rule name to a list of instantiated formulas.
        """
        initial_position = planning_problem.initial_state.position
        start = planning_problem.initial_state.time_step
        end = start + planning_horizon if planning_horizon is not None else None
        obstacle_ids = {
            obs.obstacle_id
            for obs in scenario.obstacles
            if obs.state_at_time(start) is not None
            and ((obs.prediction is not None and obs.prediction.final_time_step >= end) or end == start)
            and scenario.lanelet_network.find_lanelet_by_position([obs.state_at_time(start).position])[0]
            and np.linalg.norm(obs.state_at_time(start).position - initial_position) <= self.fov_radius
            and consider_obstacle(obs)
        }
        dt = scenario.dt
        return {rule: self._instantiate_one(rule, obstacle_ids, dt, start, end) for rule in rules}

    def _instantiate_one(
        self, rule: str, obstacle_ids: Set[int], dt: float, start: int, end: Optional[int]
    ) -> List[Formula]:
        """Instantiate a single traffic rule.

        :param rule: The name of the rule.
        :param obstacle_ids: The IDs of the obstacles to consider.
        :param dt: The time step size in s.
        :param start: The start time step.
        :param end: The end time step.
        :return: The instantiated formulas.
        """
        match rule:
            case "R_G1" | "SafeDistanceRule":
                return SafeDistanceRule(self.t_c_time_steps(dt)).instantiate(obstacle_ids, start, end)
            case "R_I5" | "EnteringVehiclesRule":
                return EnteringVehiclesRule().instantiate(obstacle_ids, start, end)
            case "R-IN1" | "StopSignRule":
                return StopSignRule(self.t_slw_time_steps(dt)).instantiate(obstacle_ids, start, end)
            case "R-IN3" | "RightBeforeLeftRule":
                return RightBeforeLeftRule(self.t_ia_time_steps(dt), self.t_ib_time_steps(dt)).instantiate(
                    obstacle_ids, start, end
                )
            case "R-IN4" | "PriorityRule":
                return PriorityRule(self.t_ia_time_steps(dt), self.t_ib_time_steps(dt)).instantiate(
                    obstacle_ids, start, end
                )
            case "R-IN5" | "TurningLeftRule":
                return TurningLeftRule(self.t_ia_time_steps(dt), self.t_ib_time_steps(dt)).instantiate(
                    obstacle_ids, start, end
                )
            case _ if rule.startswith("LTL "):
                return [Formula(rule.removeprefix("LTL "))]
            case _:
                raise ValueError(f"Unknown rule: {rule}")

    def t_c_time_steps(self, dt: float) -> int:
        """Get the number of time steps for the $t_c$ parameter.

        :param dt: The time step size in s.
        :return: The $t_c$ parameter converted to time steps.
        """
        return self._in_time_steps("t_c", self.t_c, dt)

    def t_slw_time_steps(self, dt: float) -> int:
        """Get the number of time steps for the $t_{slw}$ parameter.

        :param dt: The time step size in s.
        :return: The $t_{slw}$ parameter converted to time steps.
        """
        return self._in_time_steps("t_slw", self.t_slw, dt)

    def t_ia_time_steps(self, dt: float) -> int:
        """Get the number of time steps for the $t_{ia}$ parameter.

        :param dt: The time step size in s.
        :return: The $t_{ia}$ parameter converted to time steps.
        """
        return self._in_time_steps("t_ia", self.t_ia, dt)

    def t_ib_time_steps(self, dt: float) -> int:
        """Get the number of time steps for the $t_{ib}$ parameter.

        :param dt: The time step size in s.
        :return: The $t_{ib}$ parameter converted to time steps.
        """
        return self._in_time_steps("t_ib", self.t_ib, dt)

    @staticmethod
    def _in_time_steps(name: str, value: float, dt: float) -> int:
        """Convert a time parameter to time steps.

        :param name: The name of the parameter.
        :param value: The parameter value in s.
        :param dt: The time step size in s.
        :return: The parameter value converted to time steps.
        """
        time_steps = math.floor(value / dt)
        if time_steps != value / dt:
            warnings.warn(f"{name} is not a multiple of dt. Using floor division. {name}: {value}, dt: {dt}")
        return time_steps
