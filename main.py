import time

import commonroad_route_planner.fast_api.fast_api as route_planner
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_clcs import pycrccosy
from cr_rule_simplification import TrafficRuleFacade


def main():
    # load scenario and planning problem
    scenario_path = "scenarios/DEU_MerzenichRather-2_8814400_T-14549.xml"
    # scenario_path = "scenarios/DEU_LocationALower-11_4_T-1.xml"
    # scenario_path = "cpp/tests/scenarios/interstate_simple.xml"
    scenario, planning_problems = CommonRoadFileReader(scenario_path).open()
    planning_problem = list(planning_problems.planning_problem_dict.values())[0]

    # plan route and create clcs
    route = route_planner.generate_reference_path_from_lanelet_network_and_planning_problem(
        scenario.lanelet_network, planning_problem
    )
    reference_path = pycrccosy.Util.resample_polyline(route.reference_path, 2.0)
    ccs = pycrccosy.CurvilinearCoordinateSystem(reference_path)

    # instantiate & simplify traffic rules
    tic = time.perf_counter()
    traffic_rule_facade = TrafficRuleFacade(scenario, planning_problem, ccs)
    simplified_rules = traffic_rule_facade.get_simplified_traffic_rules(
        rules=["R_I5"],
        planning_horizon=60,
    )
    toc = time.perf_counter()
    for rule in simplified_rules:
        print(rule)
    print(f"Traffic rule instantiation and simplification took {toc - tic:0.4f} seconds")


if __name__ == "__main__":
    main()
