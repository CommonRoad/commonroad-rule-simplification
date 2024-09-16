import math
import warnings
from typing import List, Optional, Set

from commonroad.scenario.scenario import Scenario
from mltl_simplification import Formula

from cr_knowledge_extraction.traffic_rule.R_G1 import SafeDistanceRule
from cr_knowledge_extraction.traffic_rule.R_I5 import EnteringVehiclesRule


class TrafficRuleInstantiator:
    t_c: float

    def __init__(self, t_c: float = 3.0):
        self.t_c = t_c

    def instantiate(
        self, rules: List[str], scenario: Scenario, start: int = 0, end: Optional[int] = None
    ) -> List[Formula]:
        obstacle_ids = {obs.obstacle_id for obs in scenario.obstacles}
        dt = scenario.dt
        return [self._instantiate_one(rule, obstacle_ids, dt, start, end) for rule in rules]

    def _instantiate_one(self, rule: str, obstacle_ids: Set[int], dt: float, start: int, end: Optional[int]) -> Formula:
        match rule:
            case "R_G1" | "SafeDistanceRule":
                t_c_time_steps = math.floor(self.t_c / dt)
                if t_c_time_steps != self.t_c / dt:
                    warnings.warn(f"t_c is not a multiple of dt. Using floor division. t_c: {self.t_c}, dt: {dt}")
                return SafeDistanceRule(t_c_time_steps).instantiate(obstacle_ids, start, end)
            case "R_I5" | "EnteringVehiclesRule":
                return EnteringVehiclesRule().instantiate(obstacle_ids, start, end)
            case _ if rule.startswith("LTL "):
                return Formula(rule.removeprefix("LTL "))
            case _:
                raise ValueError(f"Unknown rule: {rule}")
