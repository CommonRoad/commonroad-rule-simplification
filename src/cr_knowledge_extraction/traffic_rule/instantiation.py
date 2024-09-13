from typing import List, Optional, Set

from commonroad.scenario.scenario import Scenario
from mltl_simplification import Formula

from cr_knowledge_extraction.traffic_rule.R_I5 import EnteringVehiclesRule


class TrafficRuleInstantiator:
    @staticmethod
    def instantiate(rules: List[str], scenario: Scenario, start: int = 0, end: Optional[int] = None) -> List[Formula]:
        obstacle_ids = {obs.obstacle_id for obs in scenario.obstacles}
        return [TrafficRuleInstantiator._instantiate_one(rule, obstacle_ids, start, end) for rule in rules]

    @staticmethod
    def _instantiate_one(rule: str, obstacle_ids: Set[int], start: int, end: Optional[int]) -> Formula:
        match rule:
            case "R_I5" | "EnteringVehiclesRule":
                return EnteringVehiclesRule().instantiate(obstacle_ids, start, end)
            case _ if rule.startswith("LTL "):
                return Formula(rule.removeprefix("LTL "))
            case _:
                raise ValueError(f"Unknown rule: {rule}")
