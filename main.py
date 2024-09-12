import time
from typing import Dict, Tuple

import mltl_simplification as simp
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem
from commonroad_dc import pycrccosy
from commonroad_route_planner.route_planner import RoutePlanner

from cr_knowledge_extraction import EgoParameters, ExtractionInterface, ExtractionResult


def main():
    scenario_path = "scenarios/DEU_LocationALower-11_4_T-1.xml"
    scenario, planning_problems = CommonRoadFileReader(scenario_path).open()
    planning_problem = list(planning_problems.planning_problem_dict.values())[0]
    dt = 0.08
    planning_problem.initial_state.time_step = int(planning_problem.initial_state.time_step // (dt / scenario.dt))

    # configure ego parameters
    ego_params = EgoParameters(initial_state=initialize_from_planning_problem(planning_problem))

    # plan route and create clcs
    route = RoutePlanner(scenario, planning_problem).plan_routes().retrieve_first_route()
    reference_path = pycrccosy.Util.resample_polyline(route.reference_path, 2.0)
    ccs = pycrccosy.CurvilinearCoordinateSystem(reference_path)

    # specify formula
    formula = simp.Formula("(G InFrontOf(10) & InSameLane(10)) & (G InFrontOf(12) & InSameLane(12))")
    relevant_aps = formula.relevant_aps(15)

    # extract scenario knowledge
    tic = time.perf_counter()
    extractor = ExtractionInterface(scenario_path, dt, ego_params, ccs)
    extracted_knowledge = extractor.extract(relevant_aps)
    toc = time.perf_counter()
    for time_step, result in extracted_knowledge.items():
        print("====================================")
        print(f"Time step: {time_step}")
        print(f"Positive: {result.positive_propositions}")
        print(f"Negative: {result.negative_propositions}")
        print(f"Implications: {result.implications}")
        print(f"Equivalences: {result.equivalences}")
    print(f"Extraction time: {toc - tic} seconds")

    # augment formula with knowledge
    tic = time.perf_counter()
    knowledge_sequence = convert_to_knowledge_sequence(extracted_knowledge)
    augmenter = simp.Augmenter(knowledge_sequence)
    augmented_formula = augmenter.augment(formula)
    toc = time.perf_counter()
    print(augmented_formula)
    print(f"Augmentation time: {toc - tic} seconds")


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


if __name__ == "__main__":
    main()
