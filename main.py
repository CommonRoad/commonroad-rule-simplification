import time
from typing import Dict, Tuple

import mltl_simplification as simp
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem
from commonroad_dc import pycrccosy
from commonroad_route_planner.route_planner import RoutePlanner

from cr_knowledge_extraction import EgoParameters, ExtractionInterface, ExtractionResult
from cr_knowledge_extraction.formula.formatting import format_formula_for_reach
from cr_knowledge_extraction.traffic_rule.instantiation import TrafficRuleInstantiator


def main():
    # scenario_path = "scenarios/DEU_LocationALower-11_4_T-1.xml"
    scenario_path = "cpp/tests/scenarios/interstate_simple.xml"
    scenario, planning_problems = CommonRoadFileReader(scenario_path).open()
    planning_problem = list(planning_problems.planning_problem_dict.values())[0]
    # dt = 0.08
    dt = 0.2
    planning_horizon = 15
    planning_problem.initial_state.time_step = int(planning_problem.initial_state.time_step // (dt / scenario.dt))

    # configure ego parameters
    ego_params = EgoParameters(initial_state=initialize_from_planning_problem(planning_problem))

    # plan route and create clcs
    route = RoutePlanner(scenario, planning_problem).plan_routes().retrieve_first_route()
    reference_path = pycrccosy.Util.resample_polyline(route.reference_path, 2.0)
    ccs = pycrccosy.CurvilinearCoordinateSystem(reference_path)

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
    formula = simp.Formula.conjunction(TrafficRuleInstantiator.instantiate(["R_I5"], scenario, end=planning_horizon))

    tic = time.perf_counter()
    extractor = ExtractionInterface(scenario_path, dt, ego_params, ccs)
    toc = time.perf_counter()
    print(f"Extractor initialization time: {toc - tic} seconds")

    # single pass augmentation
    print("Single pass")
    augmented = extract_and_augment(extractor, formula, planning_horizon)
    print(augmented)

    # multi pass augmentation
    extractor = ExtractionInterface(scenario_path, dt, ego_params, ccs)  # re-init to make comparison fair
    print("Multi pass")
    augmented = extract_and_augment(extractor, formula, 15, extraction_mode=1)
    augmented = extract_and_augment(extractor, augmented, 15, extraction_mode=2)
    print(augmented)

    print([format_formula_for_reach(f) for f in formula.split_at_top_level_conjunction()])
    print(format_formula_for_reach(augmented))


def extract_and_augment(extractor, formula, time_steps, verbose=True, extraction_mode=0):
    # extract scenario knowledge
    tic = time.perf_counter()
    relevant_aps = formula.relevant_aps(time_steps)
    match extraction_mode:
        case 1:
            extracted_knowledge = extractor.extract_kleene(relevant_aps)
        case 2:
            extracted_knowledge = extractor.extract_relationships(relevant_aps)
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
    augmented_formula = augmenter.augment(formula)
    toc = time.perf_counter()
    if verbose:
        print(augmented_formula)
    print(f"Augmentation time: {toc - tic} seconds")

    return augmented_formula


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
