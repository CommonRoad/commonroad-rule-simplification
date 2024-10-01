from typing import Optional

from mltl_simplification import Formula

from cr_knowledge_extraction.cr_knowledge_extraction_core import Proposition as Prop


def format_formula_for_reach(formula: Formula) -> str:
    def format_literal_reach(name: str, parameter: Optional[str]) -> str:
        if name == Prop.proposition_to_string(Prop.OTHER_ON_ACCESS_RAMP):
            name = "OnAccessRamp"
        elif name == Prop.proposition_to_string(Prop.OTHER_ON_MAIN_CARRIAGEWAY):
            name = "OnMainCarriageway"
        elif name == Prop.proposition_to_string(Prop.ON_MAIN_CARRIAGEWAY_RIGHT_LANE):
            name = "OnRightLane"
        elif name == Prop.proposition_to_string(Prop.IN_FRONT_OF):
            name = "Behind"
        if parameter:
            return f"{name}_V{parameter}"
        else:
            return name

    return formula.remove_timed_until().format_as_string(format_literal_reach)


def format_formula_for_monitor(formula: Formula) -> str:
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
        if name == Prop.proposition_to_string(Prop.IN_SAME_LANE):
            return f"in_same_lane('obsid_{parameter}', 'x_ego')"
        if name == Prop.proposition_to_string(Prop.CUT_IN):
            obs_param = f"'obsid_{parameter}'"
            return (
                f"(in_same_lane({obs_param}, 'x_ego') & !(in_single_lane({obs_param}))"
                + f" & orientation_towards({obs_param}, 'x_ego'))"
            )

        name = _to_snake(name)
        if parameter:
            return f"{name}('x_ego', 'obsid_{parameter}')"
        else:
            return f"{name}('x_ego')"

    return formula.remove_delayed_until().format_as_string(format_literal_monitor)


def _to_snake(s: str) -> str:
    return "".join("_" + c.lower() if c.isupper() else c for c in s).lstrip("_")
