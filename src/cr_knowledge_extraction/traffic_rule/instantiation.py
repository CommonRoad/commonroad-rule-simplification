import math
import warnings
from typing import Callable, Dict, List, Optional, Set

import numpy as np
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.obstacle import Obstacle
from commonroad.scenario.scenario import Scenario
from ltl_augmentation import Formula

from cr_knowledge_extraction.traffic_rule.R_G1 import SafeDistanceRule
from cr_knowledge_extraction.traffic_rule.R_I5 import EnteringVehiclesRule
from cr_knowledge_extraction.traffic_rule.R_IN1 import StopSignRule


class TrafficRuleInstantiator:
    fov_radius: float

    t_c: float
    t_slw: float

    def __init__(self, fov_radius: float = 200, t_c: float = 3.0, t_slw: float = 3.0):
        self.fov_radius = fov_radius

        self.t_c = t_c
        self.t_slw = t_slw

    def instantiate(
        self,
        rules: List[str],
        scenario: Scenario,
        planning_problem: PlanningProblem,
        time_steps: Optional[int] = None,
        consider_obstacle: Callable[[Obstacle], bool] = lambda obs: True,
    ) -> Dict[str, List[Formula]]:
        initial_position = planning_problem.initial_state.position
        start = planning_problem.initial_state.time_step
        end = start + time_steps if time_steps is not None else None
        obstacle_ids = {
            obs.obstacle_id
            for obs in scenario.obstacles
            if obs.state_at_time(start) is not None
            and np.linalg.norm(obs.state_at_time(start).position - initial_position) <= self.fov_radius
            and consider_obstacle(obs)
        }
        dt = scenario.dt
        return {rule: self._instantiate_one(rule, obstacle_ids, dt, start, end) for rule in rules}

    def _instantiate_one(
        self, rule: str, obstacle_ids: Set[int], dt: float, start: int, end: Optional[int]
    ) -> List[Formula]:
        match rule:
            case "R_G1" | "SafeDistanceRule":
                t_c_time_steps = math.floor(self.t_c / dt)
                if t_c_time_steps != self.t_c / dt:
                    warnings.warn(f"t_c is not a multiple of dt. Using floor division. t_c: {self.t_c}, dt: {dt}")
                return SafeDistanceRule(t_c_time_steps).instantiate(obstacle_ids, start, end)
            case "R_I5" | "EnteringVehiclesRule":
                return EnteringVehiclesRule().instantiate(obstacle_ids, start, end)
            case "R-IN1" | "StopSignRule":
                t_slw_time_steps = math.floor(self.t_slw / dt)
                if t_slw_time_steps != self.t_slw / dt:
                    warnings.warn(f"t_slw is not a multiple of dt. Using floor division. t_slw: {self.t_slw}, dt: {dt}")
                return StopSignRule(t_slw_time_steps).instantiate(obstacle_ids, start, end)
            case _ if rule.startswith("LTL "):
                return [Formula(rule.removeprefix("LTL "))]
            case _:
                raise ValueError(f"Unknown rule: {rule}")
