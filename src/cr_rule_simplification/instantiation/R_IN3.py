from typing import List, Optional, Set

from ltl_augmentation import Formula

from cr_rule_simplification.instantiation.not_endanger_rule import NotEndangerRule
from cr_rule_simplification.knowledge_extraction.knowledge_extraction_core import Proposition as Prop


class RightBeforeLeftRule(NotEndangerRule):
    """
    Rule R-IN3:
    G (
        (
            on_incoming_left_of(x_ego, x_o) & !relevant_traffic_light(x_ego)
            & (
                (turning_right(x_ego) & turning_right(x_o) & same_priority(x_ego, x_o, right, right))
                | (turning_right(x_ego) & turning_left(x_o) & same_priority(x_ego, x_o, right, left))
                | ...
                | (going_straight(x_ego) & going_straight(x_o) & same_priority(x_ego, x_o, straight, straight))
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
                        Formula.ap(Prop.SameRightRightPriority(), obstacle_id_str),
                    ]
                ),
                Formula.conjunction(
                    [
                        Formula.ap(Prop.TurningRight()),
                        Formula.ap(Prop.OtherTurningLeft(), obstacle_id_str),
                        Formula.ap(Prop.SameRightLeftPriority(), obstacle_id_str),
                    ]
                ),
                Formula.conjunction(
                    [
                        Formula.ap(Prop.TurningRight()),
                        Formula.ap(Prop.OtherGoingStraight(), obstacle_id_str),
                        Formula.ap(Prop.SameRightStraightPriority(), obstacle_id_str),
                    ]
                ),
                Formula.conjunction(
                    [
                        Formula.ap(Prop.TurningLeft()),
                        Formula.ap(Prop.OtherTurningRight(), obstacle_id_str),
                        Formula.ap(Prop.SameLeftRightPriority(), obstacle_id_str),
                    ]
                ),
                Formula.conjunction(
                    [
                        Formula.ap(Prop.TurningLeft()),
                        Formula.ap(Prop.OtherTurningLeft(), obstacle_id_str),
                        Formula.ap(Prop.SameLeftLeftPriority(), obstacle_id_str),
                    ]
                ),
                Formula.conjunction(
                    [
                        Formula.ap(Prop.TurningLeft()),
                        Formula.ap(Prop.OtherGoingStraight(), obstacle_id_str),
                        Formula.ap(Prop.SameLeftStraightPriority(), obstacle_id_str),
                    ]
                ),
                Formula.conjunction(
                    [
                        Formula.ap(Prop.GoingStraight()),
                        Formula.ap(Prop.OtherTurningRight(), obstacle_id_str),
                        Formula.ap(Prop.SameStraightRightPriority(), obstacle_id_str),
                    ]
                ),
                Formula.conjunction(
                    [
                        Formula.ap(Prop.GoingStraight()),
                        Formula.ap(Prop.OtherTurningLeft(), obstacle_id_str),
                        Formula.ap(Prop.SameStraightLeftPriority(), obstacle_id_str),
                    ]
                ),
                Formula.conjunction(
                    [
                        Formula.ap(Prop.GoingStraight()),
                        Formula.ap(Prop.OtherGoingStraight(), obstacle_id_str),
                        Formula.ap(Prop.SameStraightStraightPriority(), obstacle_id_str),
                    ]
                ),
            ]
        )

        condition = Formula.conjunction(
            [
                Formula.ap(Prop.OnIncomingLeftOf(), obstacle_id_str),
                Formula.negation(Formula.ap(Prop.RelevantTrafficLight())),
                turning_directions_disjunction,
            ]
        )

        consequence = self._make_consequence(obstacle_id)

        return Formula.always(Formula.implication(condition, consequence), start, end)
