from typing import List, Optional, Set

from ltl_augmentation import Formula

from cr_rule_simplification.instantiation.rule import TrafficRule
from cr_rule_simplification.knowledge_extraction.knowledge_extraction_core import Proposition as Prop


class SafeDistanceRule(TrafficRule):
    """
    Rule R_G1
    G (
    in_same_lane(x_ego, x_o)
    & in_front_of(x_ego, x_o)
    & !O[0,t_c] (
                cut_in(x_o, x_ego)
                & P(!cut_in(x_o. x_ego)))
    => keeps_safe_distance_prec(x_ego, x_o)
    )

    In Future Time:

    G(
        (
            X[t_c] in_same_lane(x_ego, x_o)
            & X[t_c] in_front_of(x_ego, x_o)
            & !F[0, t_c - 1] (!cut_in(x_o, x_ego) & X cut_in(x_o, x_ego))
        )
        =>
        X[t_c] keeps_safe_distance(x_ego, x_o)
    )
    &
    in_same_lane(x_ego, x_o) & in_front_of(x_ego, x_o) => keeps_safe_distance(x_ego, x_o)
    & (k=1 -> t_c - 1)
    (
        X[k] in_same_lane(x_ego, x_o)
        & X[k] in_front_of(x_ego, x_o)
        & !F[0, k - 1](!cut_in(x_o, x_ego) & X cut_in(x_o, x_ego))
    )
    =>
    X[k] keeps_safe_distance(x_ego, x_o)
    """

    t_c: int

    def __init__(self, t_c: int):
        self.t_c = t_c

    def instantiate(self, obstacle_ids: Set[int], start: int = 0, end: Optional[int] = None) -> List[Formula]:
        return [self.instantiate_once(obstacle_id, start, end) for obstacle_id in obstacle_ids]

    def instantiate_once(self, obstacle_id: int, start: int = 0, end: Optional[int] = None) -> Formula:
        obstacle_id = str(obstacle_id)

        if end is not None and self.t_c <= end:
            ub_globally = end - self.t_c if end is not None else None
            globally_part = Formula.always(self.instantiate_implication(obstacle_id, self.t_c), start, ub_globally)
        else:
            globally_part = Formula.true_formula()

        iteration_ub = self.t_c - 1 if end is None else min(self.t_c - 1, end)
        return Formula.conjunction(
            [self.instantiate_implication(obstacle_id, k) for k in range(1, iteration_ub + 1)]
            # simplified version for k = 0
            + [
                Formula.implication(
                    Formula.conjunction(
                        [Formula.ap(Prop.InSameLane(), obstacle_id), Formula.ap(Prop.InFrontOf(), obstacle_id)]
                    ),
                    Formula.ap(Prop.KeepsSafeDistancePrec(), obstacle_id),
                ),
                globally_part,
            ]
        )

    @staticmethod
    def instantiate_implication(obstacle_id: str, offset: int) -> Formula:
        """
        Create the core implication of the rule:

        (
            X[k] in_same_lane(x_ego, x_o)
            & X[k] in_front_of(x_ego, x_o)
            & !F[0, k - 1] (!cut_in(x_o, x_ego) & X cut_in(x_o, x_ego))
                // !F ... = G[0, k - 1] (cut_in(x_o, x_ego) | X !cut_in(x_o, x_ego))
        )
        =>
        X[k] keeps_safe_distance(x_ego, x_o)

        :param obstacle_id: ID of x_o as string
        :param offset: Time offset k
        :return: The implication formula
        """

        condition = Formula.conjunction(
            [
                Formula.next(Formula.ap(Prop.InSameLane(), obstacle_id), offset),
                Formula.next(Formula.ap(Prop.InFrontOf(), obstacle_id), offset),
                Formula.always(
                    Formula.disjunction(
                        [
                            Formula.ap(Prop.CutIn(), obstacle_id),
                            Formula.next(Formula.negation(Formula.ap(Prop.CutIn(), obstacle_id)), 1),
                        ]
                    ),
                    0,
                    offset - 1,
                ),
            ]
        )

        consequence = Formula.next(Formula.ap(Prop.KeepsSafeDistancePrec(), obstacle_id), offset)

        return Formula.implication(condition, consequence)
