import time
from typing import Dict

import mltl_simplification as simp
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_dc import pycrccosy
from commonroad_route_planner.route_planner import RoutePlanner

from cr_knowledge_extraction import ExtractionInterface, ExtractionResult


def main():
    scenario_path = "scenarios/DEU_LocationALower-11_4_T-1.xml"
    scenario, planning_problems = CommonRoadFileReader(scenario_path).open()
    planning_problem = list(planning_problems.planning_problem_dict.values())[0]

    # plan route and create clcs
    route = RoutePlanner(scenario, planning_problem).plan_routes().retrieve_first_route()
    reference_path = pycrccosy.Util.resample_polyline(route.reference_path, 2.0)
    ccs = pycrccosy.CurvilinearCoordinateSystem(reference_path)

    # specify formula
    formula = simp.Formula("(G InFrontOf(10) & InSameLane(10)) & (G InFrontOf(12) & InSameLane(12))")
    relevant_aps = formula.relevant_aps(15)

    # extract scenario knowledge
    tic = time.perf_counter()
    extractor = ExtractionInterface(scenario_path, 0.08, ccs)
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


if __name__ == "__main__":
    main()
