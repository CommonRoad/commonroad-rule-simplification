from typing import Optional, Set

from mltl_simplification import Formula

from cr_knowledge_extraction.cr_knowledge_extraction_core import Proposition as Prop
from cr_knowledge_extraction.traffic_rule.rule import TrafficRule


class EnteringVehiclesRule(TrafficRule):
    """
    Rule R_I5
    G(
        on_main_carriageway(x_ego)
        & in_front_of(x_ego, x_o)
        & on_access_ramp(x_o)
        & F(on_main_carriageways(x_o))
        -> !(! main_carriageway_right_lane(x_ego) & F(main_carriageway_right_lane(x_ego)
    )
    """

    def instantiate(self, obstacle_ids: Set[int], start: int = 0, end: Optional[int] = None) -> Formula:
        return Formula.conjunction([self.instantiate_once(obstacle_id, start, end) for obstacle_id in obstacle_ids])

    @staticmethod
    def instantiate_once(obstacle_id: int, start: int = 0, end: Optional[int] = None) -> Formula:
        obstacle_id = str(obstacle_id)

        condition = Formula.conjunction(
            [
                Formula.ap(Prop.OnMainCarriageway()),
                Formula.ap(Prop.InFrontOf(), obstacle_id),
                Formula.ap(Prop.OtherOnAccessRamp(), obstacle_id),
                Formula.eventually(Formula.ap(Prop.OtherOnMainCarriageway(), obstacle_id)),
            ]
        )

        consequence = Formula.disjunction(
            [
                Formula.ap(Prop.OnMainCarriagewayRightLane()),
                Formula.always(Formula.negation(Formula.ap(Prop.OnMainCarriagewayRightLane()))),
            ]
        )

        return Formula.always(Formula.implication(condition, consequence), start, end)
