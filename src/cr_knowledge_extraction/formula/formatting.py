from typing import Optional

from mltl_simplification import Formula

from cr_knowledge_extraction.cr_knowledge_extraction_core import Proposition as Prop


def format_formula_for_reach(formula: Formula) -> str:
    def format_literal_reach(name: str, parameters: Optional[str]) -> str:
        if name == Prop.proposition_to_string[Prop.OTHER_ON_ACCESS_RAMP]:
            name = "OnAccessRamp"
        elif name == Prop.proposition_to_string[Prop.OTHER_ON_MAIN_CARRIAGEWAY]:
            name = "OnMainCarriageway"
        elif name == Prop.proposition_to_string[Prop.ON_MAIN_CARRIAGEWAY_RIGHT_LANE]:
            name = "OnRightLane"
        elif name == Prop.proposition_to_string[Prop.IN_FRONT_OF]:
            name = "Behind"
        if parameters:
            return f"{name}_V{parameters}"
        else:
            return name

    return formula.format_as_string(format_literal_reach)


def format_formula_for_monitor(formula: Formula) -> str:
    def format_literal_monitor(name: str, parameters: Optional[str]) -> str:
        if parameters:
            return f"{name}(x_ego, {parameters})"
        else:
            return name

    return formula.format_as_string(format_literal_monitor)
