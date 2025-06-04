from typing import Dict

import crcpp
from commonroad_clcs import pycrccosy
from ltl_augmentation import Formula, KnowledgeSequence

import cr_rule_simplification.knowledge_extraction.knowledge_extraction_core as core


class KnowledgeExtractor:
    """A class for extracting knowledge from a scenario using the C++ knowledge extraction interface."""

    _cpp_extractor: core.ExtractionInterface

    def __init__(
        self, world: crcpp.World, ccs: pycrccosy.CurvilinearCoordinateSystem, ego_params: core.EgoParameters
    ) -> None:
        """Create a new KnowledgeExtractor.

        :param world: The C++ world object corresponding to the CommonRoad scenario.
        :param ccs: The curvilinear coordinate system.
        :param ego_params: The configuration parameters of the ego vehicle.
        """
        self._cpp_extractor = core.ExtractionInterface(world, ccs, ego_params)

    def extract_kleene(self, formula: Formula, planning_horizon: int) -> KnowledgeSequence:
        """Extract Kleene knowledge from the scenario.

        Kleene knowledge consists of the positive and negative propositions that hold at each time step.

        :param formula: The formula for which to extract knowledge.
        :param planning_horizon: The planning horizon.
        :return: The extracted Kleene knowledge.
        """
        relevant_aps = formula.relevant_aps(planning_horizon)
        extraction_results = self._cpp_extractor.extract_kleene(relevant_aps)
        return self._convert_extraction_results_to_knowledge_sequence(extraction_results)

    def extract_relationships(self, formula: Formula, planning_horizon: int) -> KnowledgeSequence:
        """Extract relationship knowledge from the scenario.

        Relationship knowledge consists of the implications and equivalences that hold at each time step.

        :param formula: The formula for which to extract knowledge.
        :param planning_horizon: The planning horizon.
        :return: The extracted relationship knowledge.
        """
        relevant_aps = formula.relevant_aps(planning_horizon)
        extraction_results = self._cpp_extractor.extract_relationships(relevant_aps)
        return self._convert_extraction_results_to_knowledge_sequence(extraction_results)

    @staticmethod
    def _convert_extraction_results_to_knowledge_sequence(
        extraction_results: Dict[int, core.ExtractionResult],
    ) -> KnowledgeSequence:
        """Convert the extraction results to a KnowledgeSequence.

        This is a helper method to convert the extraction results from the C++ interface to a KnowledgeSequence used in
        the ltl_augmentation package.

        :param extraction_results: The extraction results.
        :return: The knowledge sequence.
        """
        return KnowledgeSequence(
            {
                time_step: (
                    result.positive_propositions,
                    result.negative_propositions,
                    result.implications,
                    result.equivalences,
                )
                for time_step, result in extraction_results.items()
            }
        )
