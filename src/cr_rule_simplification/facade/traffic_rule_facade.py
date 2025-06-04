from typing import Callable, List, Optional, Tuple

import crcpp
import more_itertools
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.obstacle import Obstacle
from commonroad.scenario.scenario import Scenario
from commonroad_clcs import pycrccosy
from ltl_augmentation import Formula

from cr_rule_simplification.facade.configuration import Configuration
from cr_rule_simplification.formula.formatting import format_formula_for_monitor, format_formula_for_reach
from cr_rule_simplification.instantiation.traffic_rule_instantiator import TrafficRuleInstantiator
from cr_rule_simplification.knowledge_extraction import EgoParameters
from cr_rule_simplification.simplification.traffic_rule_simplifier import TrafficRuleSimplifier


class TrafficRuleFacade:
    """Facade for the traffic rule simplification pipeline.

    This class provides a high-level interface to the traffic rule simplification pipeline. It will instantiate and
    simplify traffic rules based on a given scenario and planning problem.
    """

    scenario: Scenario
    world: crcpp.World
    planning_problem: PlanningProblem
    ccs: pycrccosy.CurvilinearCoordinateSystem

    _instantiator: TrafficRuleInstantiator
    _simplifier: TrafficRuleSimplifier

    def __init__(
        self,
        scenario: Scenario,
        planning_problem: PlanningProblem,
        ccs: pycrccosy.CurvilinearCoordinateSystem,
        configuration: Configuration = Configuration(),
        world: Optional[crcpp.World] = None,
    ):
        """Create a new TrafficRuleFacade.

        :param scenario: The CommonRoad scenario.
        :param planning_problem: The planning problem.
        :param ccs: The curvilinear coordinate system.
        :param configuration: The configuration parameters for the traffic rule simplification pipeline.
        :param world: The C++ world object corresponding to the scenario. If not provided, it will be created from the
            scenario.
        """
        self.scenario = scenario
        self.planning_problem = planning_problem
        self.ccs = ccs
        self.world = world if world is not None else crcpp.World(scenario)

        self._instantiator = TrafficRuleInstantiator(
            fov_radius=configuration.fov_radius,
            t_c=configuration.t_c,
            t_slw=configuration.t_slw,
            t_ia=configuration.t_ia,
            t_ib=configuration.t_ib,
        )

        ego_params = EgoParameters(
            a_lon_min=configuration.a_lon_min,
            a_lon_max=configuration.a_lon_max,
            a_lat_min=configuration.a_lat_min,
            a_lat_max=configuration.a_lat_max,
            v_lon_min=configuration.v_lon_min,
            v_lon_max=configuration.v_lon_max,
            v_lat_min=configuration.v_lat_min,
            v_lat_max=configuration.v_lat_max,
            initial_state=self._initialize_from_planning_problem(planning_problem),
            uncertainty_p_lon=configuration.uncertainty_p_lon,
            uncertainty_p_lat=configuration.uncertainty_p_lat,
            uncertainty_v_lon=configuration.uncertainty_v_lon,
            uncertainty_v_lat=configuration.uncertainty_v_lat,
            length=configuration.length,
            width=configuration.width,
            t_react=configuration.t_react,
        )
        self._simplifier = TrafficRuleSimplifier(self.world, ego_params, self.ccs)

    def get_simplified_traffic_rules(
        self, rules: List[str], planning_horizon: int, consider_obstacle: Callable[[Obstacle], bool] = lambda obs: True
    ) -> List[Formula]:
        """Get simplified traffic rules for a given list of rule names.

        :param rules: The list of rule names.
        :param planning_horizon: The planning horizon to consider in the traffic rules.
        :param consider_obstacle: A function that returns True for each obstacle that should be considered in the
            traffic rules.
        :return: The list of simplified traffic rules.
        """
        naive_rules = self._instantiator.instantiate(
            rules, self.scenario, self.planning_problem, planning_horizon, consider_obstacle
        )
        naive_rules = list(more_itertools.flatten(naive_rules.values()))

        simplified_rules = self._simplifier.simplify(naive_rules, planning_horizon)

        return simplified_rules

    def get_simplified_traffic_rules_for_reach(
        self, rules: List[str], planning_horizon: int, consider_obstacle: Callable[[Obstacle], bool] = lambda obs: True
    ) -> List[str]:
        """Get simplified traffic rules for a given list of rule names formatted for use with CommonRoad-Reach-Flow.

        :param rules: The list of rule names.
        :param planning_horizon: The planning horizon to consider in the traffic rules.
        :param consider_obstacle: A function that returns True for each obstacle that should be considered in the
            traffic rules.
        :return: The list of simplified traffic rules.
        """
        simplified_rules = self.get_simplified_traffic_rules(rules, planning_horizon, consider_obstacle)
        return [format_formula_for_reach(rule) for rule in simplified_rules]

    def get_simplified_traffic_rules_for_monitor(
        self, rules: List[str], planning_horizon: int, consider_obstacle: Callable[[Obstacle], bool] = lambda obs: True
    ) -> List[str]:
        """Get simplified traffic rules for a given list of rule names formatted for use with the CommonRoad-Monitor.

        :param rules: The list of rule names.
        :param planning_horizon: The planning horizon to consider in the traffic rules.
        :param consider_obstacle: A function that returns True for each obstacle that should be considered in the
            traffic rules.
        :return: The list of simplified traffic rules.
        """
        simplified_rules = self.get_simplified_traffic_rules(rules, planning_horizon, consider_obstacle)
        return [format_formula_for_monitor(rule) for rule in simplified_rules]

    @staticmethod
    def _initialize_from_planning_problem(
        planning_problem: PlanningProblem,
    ) -> Tuple[int, float, float, float, float, float]:
        """Unpack the values from the initial state of a planning problem.

        :param planning_problem: The planning problem.
        :return: The initial state as a tuple of time step, x position, y position, velocity, acceleration, and
            orientation.
        """
        state = planning_problem.initial_state
        return (
            state.time_step,
            state.position[0],
            state.position[1],
            state.velocity,
            state.acceleration,
            state.orientation,
        )
