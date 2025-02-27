from typing import Optional

from ltl_augmentation import Formula

from cr_rule_simplification.knowledge_extraction.knowledge_extraction_core import Proposition as Prop


def format_formula_for_reach(formula: Formula) -> str:
    """Format a formula for use with CommonRoad-Reach-Flow.

    :param formula: The formula.
    :return: The formatted formula.
    """

    def format_literal_reach(name: str, parameter: Optional[str]) -> str:
        if name == Prop.proposition_to_string(Prop.OTHER_ON_ACCESS_RAMP):
            name = "OnAccessRamp"
        elif name == Prop.proposition_to_string(Prop.OTHER_ON_MAIN_CARRIAGEWAY):
            name = "OnMainCarriageway"
        elif name == Prop.proposition_to_string(Prop.IN_FRONT_OF):
            name = "Behind"
        if parameter:
            return f"{name}_V{parameter}"
        else:
            return name

    return formula.remove_timed_until().format_as_string(format_literal_reach)


def format_formula_for_monitor(formula: Formula) -> str:
    """Format a formula for use with the CommonRoad-Monitor.

    :param formula: The formula.
    :return: The formatted formula.
    """

    def format_literal_monitor(name: str, parameter: Optional[str]) -> str:
        if name in {"true", "false"}:
            return name
        if name == Prop.proposition_to_string(Prop.OTHER_ON_ACCESS_RAMP):
            return f"on_lanelet_with_type('obsid_{parameter}', 'accessramp')"
        if name == Prop.proposition_to_string(Prop.OTHER_ON_MAIN_CARRIAGEWAY):
            return f"on_lanelet_with_type('obsid_{parameter}', 'maincarriageway')"
        if name == Prop.proposition_to_string(Prop.ON_MAIN_CARRIAGEWAY):
            return "on_lanelet_with_type('x_ego', 'maincarriageway')"
        if name == Prop.proposition_to_string(Prop.ON_MAIN_CARRIAGEWAY_RIGHT_LANE):
            return "rightmost_lane_with_type('x_ego', 'maincarriageway')"
        if name == Prop.proposition_to_string(Prop.ON_MAIN_CARRIAGEWAY_LEFT_LANE):
            return "leftmost_lane_with_type('x_ego', 'maincarriageway')"
        if name == Prop.proposition_to_string(Prop.IN_SAME_LANE):
            return f"in_same_lane('obsid_{parameter}', 'x_ego')"
        if name == Prop.proposition_to_string(Prop.CUT_IN):
            obs_param = f"'obsid_{parameter}'"
            return (
                f"(in_same_lane({obs_param}, 'x_ego') & !(in_single_lane({obs_param}))"
                + f" & orientation_towards({obs_param}, 'x_ego'))"
            )
        if name == Prop.proposition_to_string(Prop.KEEPS_SAFE_DISTANCE_PREC):
            # We need to rewrite the safe distance predicate here, because its implementation in the environment model
            # does not match its definition from the paper. According to the paper, it should be false, when the
            # following vehicle is not behind the leading vehicle, but the implementation sets it to true
            obs_param = f"'obsid_{parameter}'"
            return f"(in_front_of('x_ego', {obs_param}) & keeps_safe_distance_prec('x_ego', {obs_param}))"
        if name == Prop.proposition_to_string(Prop.AT_STOP_SIGN):
            return "at_traffic_sign('x_ego', 'stop')"
        if name == Prop.proposition_to_string(Prop.OTHER_IN_INTERSECTION_CONFLICT_AREA):
            return f"in_intersection_conflict_area('obsid_{parameter}', 'x_ego')"
        if name == Prop.proposition_to_string(Prop.ON_ONCOMING_OF):
            return f"on_oncoming_of('obsid_{parameter}', 'x_ego')"
        if name == Prop.proposition_to_string(Prop.IN_INTERSECTION):
            return "on_lanelet_with_type('x_ego', 'intersection')"

        if name == Prop.proposition_to_string(Prop.TURNING_LEFT):
            return "turning('x_ego', 'left')"
        if name == Prop.proposition_to_string(Prop.TURNING_RIGHT):
            return "turning('x_ego', 'right')"
        if name == Prop.proposition_to_string(Prop.GOING_STRAIGHT):
            return "turning('x_ego', 'straight')"
        if name == Prop.proposition_to_string(Prop.OTHER_TURNING_LEFT):
            return f"turning('obsid_{parameter}', 'left')"
        if name == Prop.proposition_to_string(Prop.OTHER_TURNING_RIGHT):
            return f"turning('obsid_{parameter}', 'right')"
        if name == Prop.proposition_to_string(Prop.OTHER_GOING_STRAIGHT):
            return f"turning('obsid_{parameter}', 'straight')"

        if name == Prop.proposition_to_string(Prop.SAME_LEFT_LEFT_PRIORITY):
            return f"same_priority('x_ego', 'obsid_{parameter}', 'left', 'left')"
        if name == Prop.proposition_to_string(Prop.SAME_LEFT_RIGHT_PRIORITY):
            return f"same_priority('x_ego', 'obsid_{parameter}', 'left', 'right')"
        if name == Prop.proposition_to_string(Prop.SAME_LEFT_STRAIGHT_PRIORITY):
            return f"same_priority('x_ego', 'obsid_{parameter}', 'left', 'straight')"
        if name == Prop.proposition_to_string(Prop.SAME_RIGHT_LEFT_PRIORITY):
            return f"same_priority('x_ego', 'obsid_{parameter}', 'right', 'left')"
        if name == Prop.proposition_to_string(Prop.SAME_RIGHT_RIGHT_PRIORITY):
            return f"same_priority('x_ego', 'obsid_{parameter}', 'right', 'right')"
        if name == Prop.proposition_to_string(Prop.SAME_RIGHT_STRAIGHT_PRIORITY):
            return f"same_priority('x_ego', 'obsid_{parameter}', 'right', 'straight')"
        if name == Prop.proposition_to_string(Prop.SAME_STRAIGHT_LEFT_PRIORITY):
            return f"same_priority('x_ego', 'obsid_{parameter}', 'straight', 'left')"
        if name == Prop.proposition_to_string(Prop.SAME_STRAIGHT_RIGHT_PRIORITY):
            return f"same_priority('x_ego', 'obsid_{parameter}', 'straight', 'right')"
        if name == Prop.proposition_to_string(Prop.SAME_STRAIGHT_STRAIGHT_PRIORITY):
            return f"same_priority('x_ego', 'obsid_{parameter}', 'straight', 'straight')"

        if name == Prop.proposition_to_string(Prop.HAS_LEFT_LEFT_PRIORITY):
            return f"has_priority('x_ego', 'obsid_{parameter}', 'left', 'left')"
        if name == Prop.proposition_to_string(Prop.HAS_LEFT_RIGHT_PRIORITY):
            return f"has_priority('x_ego', 'obsid_{parameter}', 'left', 'right')"
        if name == Prop.proposition_to_string(Prop.HAS_LEFT_STRAIGHT_PRIORITY):
            return f"has_priority('x_ego', 'obsid_{parameter}', 'left', 'straight')"
        if name == Prop.proposition_to_string(Prop.HAS_RIGHT_LEFT_PRIORITY):
            return f"has_priority('x_ego', 'obsid_{parameter}', 'right', 'left')"
        if name == Prop.proposition_to_string(Prop.HAS_RIGHT_RIGHT_PRIORITY):
            return f"has_priority('x_ego', 'obsid_{parameter}', 'right', 'right')"
        if name == Prop.proposition_to_string(Prop.HAS_RIGHT_STRAIGHT_PRIORITY):
            return f"has_priority('x_ego', 'obsid_{parameter}', 'right', 'straight')"
        if name == Prop.proposition_to_string(Prop.HAS_STRAIGHT_LEFT_PRIORITY):
            return f"has_priority('x_ego', 'obsid_{parameter}', 'straight', 'left')"
        if name == Prop.proposition_to_string(Prop.HAS_STRAIGHT_RIGHT_PRIORITY):
            return f"has_priority('x_ego', 'obsid_{parameter}', 'straight', 'right')"
        if name == Prop.proposition_to_string(Prop.HAS_STRAIGHT_STRAIGHT_PRIORITY):
            return f"has_priority('x_ego', 'obsid_{parameter}', 'straight', 'straight')"

        if name == Prop.proposition_to_string(Prop.OTHER_HAS_LEFT_LEFT_PRIORITY):
            return f"has_priority('obsid_{parameter}', 'x_ego', 'left', 'left')"
        if name == Prop.proposition_to_string(Prop.OTHER_HAS_LEFT_RIGHT_PRIORITY):
            return f"has_priority('obsid_{parameter}', 'x_ego', 'left', 'right')"
        if name == Prop.proposition_to_string(Prop.OTHER_HAS_LEFT_STRAIGHT_PRIORITY):
            return f"has_priority('obsid_{parameter}', 'x_ego', 'left', 'straight')"
        if name == Prop.proposition_to_string(Prop.OTHER_HAS_RIGHT_LEFT_PRIORITY):
            return f"has_priority('obsid_{parameter}', 'x_ego', 'right', 'left')"
        if name == Prop.proposition_to_string(Prop.OTHER_HAS_RIGHT_RIGHT_PRIORITY):
            return f"has_priority('obsid_{parameter}', 'x_ego', 'right', 'right')"
        if name == Prop.proposition_to_string(Prop.OTHER_HAS_RIGHT_STRAIGHT_PRIORITY):
            return f"has_priority('obsid_{parameter}', 'x_ego', 'right', 'straight')"
        if name == Prop.proposition_to_string(Prop.OTHER_HAS_STRAIGHT_LEFT_PRIORITY):
            return f"has_priority('obsid_{parameter}', 'x_ego', 'straight', 'left')"
        if name == Prop.proposition_to_string(Prop.OTHER_HAS_STRAIGHT_RIGHT_PRIORITY):
            return f"has_priority('obsid_{parameter}', 'x_ego', 'straight', 'right')"
        if name == Prop.proposition_to_string(Prop.OTHER_HAS_STRAIGHT_STRAIGHT_PRIORITY):
            return f"has_priority('obsid_{parameter}', 'x_ego', 'straight', 'straight')"

        name = _to_snake(name)
        if parameter:
            return f"{name}('x_ego', 'obsid_{parameter}')"
        else:
            return f"{name}('x_ego')"

    return formula.remove_delayed_until().format_as_string(format_literal_monitor)


def _to_snake(s: str) -> str:
    """Convert a camel case string to snake case.

    :param s: The camel case string.
    :return: The snake case string.
    """
    return "".join("_" + c.lower() if c.isupper() else c for c in s).lstrip("_")
