from typing import Dict

import crcpp
from commonroad_dc import pycrccosy
from ltl_augmentation import Formula, KnowledgeSequence

import cr_rule_simplification.knowledge_extraction.knowledge_extraction_core as core


class KnowledgeExtractor:
    _cpp_extractor: core.ExtractionInterface

    def __init__(
        self, world: crcpp.World, ego_params: core.EgoParameters, ccs: pycrccosy.CurvilinearCoordinateSystem
    ) -> None:
        self._cpp_extractor = core.ExtractionInterface(world, ego_params, ccs.reference_path_original())

    def extract_kleene(self, formula: Formula, planning_horizon: int) -> KnowledgeSequence:
        relevant_aps = formula.relevant_aps(planning_horizon)
        extraction_results = self._cpp_extractor.extract_kleene(relevant_aps)
        return self._convert_extraction_results_to_knowledge_sequence(extraction_results)

    def extract_relationships(self, formula: Formula, planning_horizon: int) -> KnowledgeSequence:
        relevant_aps = formula.relevant_aps(planning_horizon)
        extraction_results = self._cpp_extractor.extract_relationships(relevant_aps)
        return self._convert_extraction_results_to_knowledge_sequence(extraction_results)

    @staticmethod
    def _convert_extraction_results_to_knowledge_sequence(
        extraction_results: Dict[int, core.ExtractionResult]
    ) -> KnowledgeSequence:
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
