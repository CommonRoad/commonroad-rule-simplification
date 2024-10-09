import abc

from ltl_augmentation import Formula

from cr_knowledge_extraction.cr_knowledge_extraction_core import Proposition as Prop
from cr_knowledge_extraction.traffic_rule.rule import TrafficRule


class NotEndangerRule(TrafficRule, abc.ABC):
    _t_ia: int
    _t_ib: int

    def __init__(self, t_ia: int, t_ib: int):
        self._t_ia = t_ia
        self._t_ib = t_ib

    def _make_consequence(self, obstacle_id: int) -> Formula:
        return Formula.disjunction(
            [
                Formula.always(Formula.ap(Prop.NotEndanger())),
                Formula.negation(Formula.ap(Prop.IN_INTERSECTION)),
            ]
        )

    def _make_not_endanger_intersection(self, obstacle_id: int) -> Formula:
        t_ib_part = Formula.implication(
            Formula.ap(Prop.InIntersectionConflictArea(), obstacle_id),
            Formula.conjunction(
                [
                    Formula.negation(Formula.ap(Prop.CausesBrakingIntersection(), obstacle_id)),
                    Formula.negation(
                        Formula.eventually(
                            Formula.ap(Prop.OtherInIntersectionConflictArea(), obstacle_id), 0, self._t_ib
                        )
                    ),
                ]
            ),
        )
        t_ia_part = Formula.implication(
            Formula.ap(Prop.OtherInIntersectionConflictArea(), obstacle_id),
            Formula.negation(
                Formula.eventually(Formula.ap(Prop.InIntersectionConflictArea(), obstacle_id), 0, self._t_ia)
            ),
        )

        return Formula.conjunction([t_ia_part, t_ib_part])
