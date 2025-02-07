from typing import List, Optional, Set

from ltl_augmentation import Formula

from cr_rule_simplification.instantiation.not_endanger_rule import NotEndangerRule
from cr_rule_simplification.knowledge_extraction.knowledge_extraction_core import Proposition as Prop


class TurningLeftRule(NotEndangerRule):
    """
    Rule R-IN5 from Maierhofer et al. (2022):

    G (
        (
            turning_left(x_ego) & on_oncoming_of(x_o, x_ego)
            & (
                (turning_right(x_o) & !has_priority(x_ego, x_o, left, right))
                | (going_straight(x_o) & !has_priority(x_ego, x_o, left, straight))
            )
        )
        =>
        (
            G !endanger_intersection(x_ego, x_o)
            | !in_intersection(x_ego)
        )
    )
    """

    def __init__(self, t_ia: int, t_ib: int):
        super().__init__(t_ia, t_ib)

    def instantiate(self, obstacle_ids: Set[int], start: int = 0, end: Optional[int] = None) -> List[Formula]:
        return [self.instantiate_once(obstacle_id, start, end) for obstacle_id in obstacle_ids]

    def instantiate_once(self, obstacle_id: int, start: int = 0, end: Optional[int] = None) -> Formula:
        obstacle_id_str = str(obstacle_id)

        turning_directions_disjunction = Formula.disjunction(
            [
                Formula.conjunction(
                    [
                        Formula.ap(Prop.OtherTurningRight(), obstacle_id_str),
                        Formula.negation(Formula.ap(Prop.HasLeftRightPriority(), obstacle_id_str)),
                    ]
                ),
                Formula.conjunction(
                    [
                        Formula.ap(Prop.OtherGoingStraight(), obstacle_id_str),
                        Formula.negation(Formula.ap(Prop.HasLeftStraightPriority(), obstacle_id_str)),
                    ]
                ),
            ]
        )

        condition = Formula.conjunction(
            [
                Formula.ap(Prop.TurningLeft()),
                Formula.ap(Prop.OnOncomingOf(), obstacle_id_str),
                turning_directions_disjunction,
            ]
        )

        consequence = self._make_consequence(obstacle_id)

        return Formula.always(Formula.implication(condition, consequence), start, end)
