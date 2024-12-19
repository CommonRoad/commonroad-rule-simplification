import abc

from ltl_augmentation import Formula

from cr_rule_simplification.instantiation.rule import TrafficRule
from cr_rule_simplification.knowledge_extraction.knowledge_extraction_core import Proposition as Prop


class NotEndangerRule(TrafficRule, abc.ABC):
    _t_ia: int
    _t_ib: int

    def __init__(self, t_ia: int, t_ib: int):
        self._t_ia = t_ia
        self._t_ib = t_ib

    def _make_consequence(self, obstacle_id: int) -> Formula:
        return Formula.disjunction(
            [
                Formula.always(self._make_not_endanger_intersection(obstacle_id)),
                Formula.negation(Formula.ap(Prop.InIntersection())),
            ]
        )

    def _make_not_endanger_intersection(self, obstacle_id: int) -> Formula:
        obstacle_id = str(obstacle_id)
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
