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
from cr_knowledge_extraction.traffic_rule.R_IN3 import RightBeforeLeftRule


class TrafficRuleInstantiator:
    fov_radius: float

    t_c: float
    t_slw: float
    t_ia: float
    t_ib: float

    def __init__(
        self, fov_radius: float = 200, t_c: float = 3.0, t_slw: float = 3.0, t_ia: float = 0.5, t_ib: float = 1.0
    ):
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
            and obs.prediction.final_time_step >= end
            and scenario.lanelet_network.find_lanelet_by_position([obs.state_at_time(start).position])[0]
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
                return SafeDistanceRule(self.t_c_time_steps(dt)).instantiate(obstacle_ids, start, end)
            case "R_I5" | "EnteringVehiclesRule":
                return EnteringVehiclesRule().instantiate(obstacle_ids, start, end)
            case "R-IN1" | "StopSignRule":
                return StopSignRule(self.t_slw_time_steps(dt)).instantiate(obstacle_ids, start, end)
            case "R-IN3" | "RightBeforeLeftRule":
                return RightBeforeLeftRule(self.t_ia_time_steps(dt), self.t_ib_time_steps(dt)).instantiate(
                    obstacle_ids, start, end
                )
            case _ if rule.startswith("LTL "):
                return [Formula(rule.removeprefix("LTL "))]
            case _:
                raise ValueError(f"Unknown rule: {rule}")

    def t_c_time_steps(self, dt: float) -> int:
        return self._in_time_steps("t_c", self.t_c, dt)

    def t_slw_time_steps(self, dt: float) -> int:
        return self._in_time_steps("t_slw", self.t_slw, dt)

    def t_ia_time_steps(self, dt: float) -> int:
        return self._in_time_steps("t_ia", self.t_ia, dt)

    def t_ib_time_steps(self, dt: float) -> int:
        return self._in_time_steps("t_ib", self.t_ib, dt)

    @staticmethod
    def _in_time_steps(name: str, value: float, dt: float) -> int:
        time_steps = math.floor(value / dt)
        if time_steps != value / dt:
            warnings.warn(f"{name} is not a multiple of dt. Using floor division. {name}: {value}, dt: {dt}")
        return time_steps
