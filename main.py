import time
from typing import Dict, Tuple

import crcpp
import mltl_simplification as simp
import more_itertools
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem
from commonroad_dc import pycrccosy
from commonroad_route_planner.route_planner import RoutePlanner
from mltl_simplification import Formula

from cr_knowledge_extraction import EgoParameters, ExtractionInterface, ExtractionResult
from cr_knowledge_extraction.formula.formatting import format_formula_for_reach
from cr_knowledge_extraction.traffic_rule.instantiation import TrafficRuleInstantiator


def main():
    # scenario_path = "scenarios/DEU_LocationALower-11_4_T-1.xml"
    # scenario_path = "cpp/tests/scenarios/interstate_simple.xml"
    scenario_path = "scenarios/DEU_MerzenichRather-2_8814400_T-14549.xml"
    scenario, planning_problems = CommonRoadFileReader(scenario_path).open()
    planning_problem = list(planning_problems.planning_problem_dict.values())[0]
    dt = 0.08
    # dt = 0.2
    planning_horizon = 30
    planning_problem.initial_state.time_step = int(planning_problem.initial_state.time_step // (dt / scenario.dt))
    world = crcpp.World(scenario)

    # configure ego parameters
    ego_params = EgoParameters(initial_state=initialize_from_planning_problem(planning_problem))

    # plan route and create clcs
    route = RoutePlanner(scenario, planning_problem).plan_routes().retrieve_first_route()
    reference_path = pycrccosy.Util.resample_polyline(route.reference_path, 2.0)
    # ccs = pycrccosy.CurvilinearCoordinateSystem(reference_path)

    # specify formula
    # formula = simp.Formula("(G InFrontOf(10) & InSameLane(10)) & (G InFrontOf(12) & InSameLane(12))")
    # formula = simp.Formula(
    #     """
    #     (G OnMainCarriageway & InFrontOf(103) & OtherOnAccessRamp(103) & (F OtherOnMainCarriageway(103)) ->
    #         !(!OnMainCarriagewayRightLane & F OnMainCarriagewayRightLane)) &
    #     (G OnMainCarriageway & InFrontOf(102) & OtherOnAccessRamp(102) & (F OtherOnMainCarriageway(102)) ->
    #         !(!OnMainCarriagewayRightLane & F OnMainCarriagewayRightLane))
    #     """
    # )
    formulas = list(
        more_itertools.flatten(
            TrafficRuleInstantiator()
            .instantiate(["R_G1", "R_I5"], scenario, planning_problem, time_steps=planning_horizon)
            .values()
        )
    )
    print(simp.Formula.conjunction(formulas))

    tic = time.perf_counter()
    extractor = ExtractionInterface(world, ego_params, reference_path)
    toc = time.perf_counter()
    print(f"Extractor initialization time: {toc - tic} seconds")

    # single pass augmentation
    print("Single pass")
    augmented = extract_and_augment(extractor, formulas, planning_horizon)
    print(simp.Formula.conjunction(augmented))

    # multi pass augmentation
    extractor = ExtractionInterface(world, ego_params, reference_path)  # re-init to make comparison fair
    print("Multi pass")
    augmented = extract_and_augment(extractor, formulas, planning_horizon, extraction_mode=1)
    augmented = extract_and_augment(extractor, augmented, planning_horizon, extraction_mode=2)
    print(simp.Formula.conjunction(augmented))

    print([format_formula_for_reach(f) for f in formulas])
    print([format_formula_for_reach(f) for f in augmented])


def extract_and_augment(extractor, formulas, time_steps, verbose=True, extraction_mode=0):
    # extract scenario knowledge
    tic = time.perf_counter()
    relevant_aps = Formula.conjunction(formulas).relevant_aps(time_steps)
    match extraction_mode:
        case 1:
            extracted_knowledge = extractor.extract_kleene(relevant_aps)
        case 2:
            extracted_knowledge = extractor.extract_relationships(relevant_aps)
        case 3:
            extracted_knowledge = extractor.extract_equivalences(relevant_aps)
        case _:
            extracted_knowledge = extractor.extract_all(relevant_aps)
    toc = time.perf_counter()
    if verbose:
        print_knowledge(extracted_knowledge)
    print(f"Extraction time: {toc - tic} seconds")

    # augment formula with knowledge
    tic = time.perf_counter()
    knowledge_sequence = convert_to_knowledge_sequence(extracted_knowledge)
    augmenter = simp.Augmenter(knowledge_sequence)
    augmented_formulas = [aug for formula in formulas if not (aug := augmenter.augment(formula)).is_true()]
    toc = time.perf_counter()
    if verbose:
        print(simp.Formula.conjunction(augmented_formulas))
    print(f"Augmentation time: {toc - tic} seconds")

    return augmented_formulas


def convert_to_knowledge_sequence(extracted_knowledge: Dict[int, ExtractionResult]) -> simp.KnowledgeSequence:
    knowledge = {
        time_step: (
            result.positive_propositions,
            result.negative_propositions,
            result.implications,
            result.equivalences,
        )
        for time_step, result in extracted_knowledge.items()
    }
    return simp.KnowledgeSequence(knowledge)


def initialize_from_planning_problem(
    planning_problem: PlanningProblem,
) -> Tuple[int, float, float, float, float, float]:
    state = planning_problem.initial_state
    # there is no acceleration in the planning problem, so we set it to 0
    return state.time_step, state.position[0], state.position[1], state.velocity, 0, state.orientation


def print_knowledge(extracted_knowledge):
    for time_step, result in sorted(extracted_knowledge.items()):
        print("====================================")
        print(f"Time step: {time_step}")
        print(f"Positive: {', '.join(result.positive_propositions)}")
        print(f"Negative: {', '.join(result.negative_propositions)}")
        print(f"Implications: {', '.join(f'{a} -> {b}' for a, b in result.implications)}")
        print(f"Equivalences: {', '.join(f'{a} <-> {b}' for a, b in result.equivalences)}")


if __name__ == "__main__":
    main()
