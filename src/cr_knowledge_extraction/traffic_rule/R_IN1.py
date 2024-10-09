from typing import List, Optional, Set

from ltl_augmentation import Formula

from cr_knowledge_extraction.cr_knowledge_extraction_core import Proposition as Prop
from cr_knowledge_extraction.traffic_rule.rule import TrafficRule


class StopSignRule(TrafficRule):
    """
    Rule R-IN1:
    G(
        stop_line_in_front(x_ego) & X !stop_line_in_front(x_ego)
        & at_traffic_sign(x_ego, stop)
        & !relevant_traffic_light(x_ego)
        -> O G[0, t_slw] stop_line_in_front(x_ego) & in_standstill(x_ego)
    )

    In Future Time:

    (
        !stop_line_in_front(x_ego) | X stop_line_in_front(x_ego)
        | !at_traffic_sign(x_ego, stop)
        | relevant_traffic_light(x_ego)
    )
    W (weak until)
    (
        G[0, t_slw] stop_line_in_front(x_ego) & in_standstill(x_ego)
    )
    """

    t_slw: int

    def __init__(self, t_slw: int):
        self.t_slw = t_slw

    def instantiate(self, obstacle_ids: Set[int], start: int = 0, end: Optional[int] = None) -> List[Formula]:
        release_condition = Formula.always(
            Formula.conjunction(
                [
                    Formula.ap(Prop.StopLineInFront()),
                    Formula.ap(Prop.InStandstill()),
                ]
            ),
            0,
            self.t_slw,
        )

        hold_condition = Formula.disjunction(
            [
                Formula.negation(Formula.ap(Prop.StopLineInFront())),
                Formula.next(Formula.ap(Prop.StopLineInFront())),
                Formula.negation(Formula.ap(Prop.AtStopSign())),
                Formula.ap(Prop.RelevantTrafficLight()),
            ]
        )

        return [
            Formula.disjunction(
                [
                    Formula.always(hold_condition, start, end),
                    Formula.until(hold_condition, release_condition, start, end),
                ]
            )
        ]
