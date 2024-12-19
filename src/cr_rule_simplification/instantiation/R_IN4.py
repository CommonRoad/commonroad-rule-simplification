from typing import List, Optional, Set

from ltl_augmentation import Formula

from cr_rule_simplification.cr_knowledge_extraction_core import Proposition as Prop
from cr_rule_simplification.instantiation.not_endanger_rule import NotEndangerRule


class PriorityRule(NotEndangerRule):
    """
    Rule R-IN4:
    G (
        (
            (turning_right(x_ego) & turning_right(x_o) & has_priority(x_o, x_ego, right, right))
            | ...
            | (
                turning_left(x_ego) & turning_right(x_o)
                & has_priority(x_o, x_ego, right, left) & !on_oncoming_of(x_o, x_ego)
            )
            | (
                turning_left(x_ego) & going_straight(x_o)
                & has_priority(x_o, x_ego, straight, left) & !on_oncoming_of(x_o, x_ego)
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
                        Formula.ap(Prop.TurningRight()),
                        Formula.ap(Prop.OtherTurningRight(), obstacle_id_str),
                        Formula.ap(Prop.OtherHasRightRightPriority(), obstacle_id_str),
                    ]
                ),
                Formula.conjunction(
                    [
                        Formula.ap(Prop.TurningRight()),
                        Formula.ap(Prop.OtherTurningLeft(), obstacle_id_str),
                        Formula.ap(Prop.OtherHasLeftRightPriority(), obstacle_id_str),
                    ]
                ),
                Formula.conjunction(
                    [
                        Formula.ap(Prop.TurningRight()),
                        Formula.ap(Prop.OtherGoingStraight(), obstacle_id_str),
                        Formula.ap(Prop.OtherHasStraightRightPriority(), obstacle_id_str),
                    ]
                ),
                Formula.conjunction(
                    [
                        Formula.ap(Prop.TurningLeft()),
                        Formula.ap(Prop.OtherTurningRight(), obstacle_id_str),
                        Formula.ap(Prop.OtherHasRightLeftPriority(), obstacle_id_str),
                        Formula.negation(Formula.ap(Prop.OnOncomingOf(), obstacle_id_str)),
                    ]
                ),
                Formula.conjunction(
                    [
                        Formula.ap(Prop.TurningLeft()),
                        Formula.ap(Prop.OtherTurningLeft(), obstacle_id_str),
                        Formula.ap(Prop.OtherHasLeftLeftPriority(), obstacle_id_str),
                    ]
                ),
                Formula.conjunction(
                    [
                        Formula.ap(Prop.TurningLeft()),
                        Formula.ap(Prop.OtherGoingStraight(), obstacle_id_str),
                        Formula.ap(Prop.OtherHasStraightLeftPriority(), obstacle_id_str),
                        Formula.negation(Formula.ap(Prop.OnOncomingOf(), obstacle_id_str)),
                    ]
                ),
                Formula.conjunction(
                    [
                        Formula.ap(Prop.GoingStraight()),
                        Formula.ap(Prop.OtherTurningRight(), obstacle_id_str),
                        Formula.ap(Prop.OtherHasRightStraightPriority(), obstacle_id_str),
                    ]
                ),
                Formula.conjunction(
                    [
                        Formula.ap(Prop.GoingStraight()),
                        Formula.ap(Prop.OtherTurningLeft(), obstacle_id_str),
                        Formula.ap(Prop.OtherHasLeftStraightPriority(), obstacle_id_str),
                    ]
                ),
                Formula.conjunction(
                    [
                        Formula.ap(Prop.GoingStraight()),
                        Formula.ap(Prop.OtherGoingStraight(), obstacle_id_str),
                        Formula.ap(Prop.OtherHasStraightStraightPriority(), obstacle_id_str),
                    ]
                ),
            ]
        )

        consequence = self._make_consequence(obstacle_id)

        return Formula.always(Formula.implication(turning_directions_disjunction, consequence), start, end)
