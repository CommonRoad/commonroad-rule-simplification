from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_dc import pycrccosy
from commonroad_route_planner.route_planner import RoutePlanner

from cr_knowledge_extraction import ExtractionInterface, hello


def main():
    scenario_path = "scenarios/DEU_LocationALower-11_4_T-1.xml"
    scenario, planning_problems = CommonRoadFileReader(scenario_path).open()
    planning_problem = list(planning_problems.planning_problem_dict.values())[0]

    # plan route and create clcs
    route = RoutePlanner(scenario, planning_problem).plan_routes().retrieve_first_route()
    reference_path = pycrccosy.Util.resample_polyline(route.reference_path, 2.0)
    ccs = pycrccosy.CurvilinearCoordinateSystem(reference_path)

    extractor = ExtractionInterface(scenario_path, ccs)
    results = extractor.extract(
        {
            0: ["InFrontOf(10)", "InFrontOf(11)", "InFrontOf(12)", "InFrontOf(13)", "InFrontOf(14)", "InFrontOf(15)"],
            1: [
                "InSameLane(10)",
                "InSameLane(11)",
                "InSameLane(12)",
                "InSameLane(13)",
                "InSameLane(14)",
                "InSameLane(15)",
            ],
        }
    )
    for time_step, result in results.items():
        print("====================================")
        print(f"Time step: {time_step}")
        print(f"Positive: {result.positive_propositions}")
        print(f"Negative: {result.negative_propositions}")
        print(f"Implications: {result.implications}")
        print(f"Equivalences: {result.equivalences}")

    print(hello.hello())
    print(hello.hello_cpp())


if __name__ == "__main__":
    main()
