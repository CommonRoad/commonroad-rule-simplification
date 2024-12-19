from typing import Iterable, List

import crcpp
from commonroad_dc import pycrccosy
from ltl_augmentation import Augmenter, Formula, KnowledgeSequence

import cr_rule_simplification.cr_knowledge_extraction_core as core
from cr_rule_simplification.knowledge_extraction.knowledge_extractor import KnowledgeExtractor


class TrafficRuleSimplifier:
    _knowledge_extractor: KnowledgeExtractor

    def __init__(
        self, world: crcpp.World, ego_params: core.EgoParameters, ccs: pycrccosy.CurvilinearCoordinateSystem
    ) -> None:
        self._knowledge_extractor = KnowledgeExtractor(world, ego_params, ccs)

    def simplify(self, rules: Iterable[Formula], planning_horizon: int) -> List[Formula]:
        # First pass: only use Kleene knowledge
        combined_rule = Formula.conjunction(list(rules))
        kleene_knowledge = self._knowledge_extractor.extract_kleene(combined_rule, planning_horizon)
        augmented_with_kleene = self._augment_formulas(rules, kleene_knowledge)

        # Second pass: add knowledge about relationships
        combined_rule = Formula.conjunction(augmented_with_kleene)
        relationship_knowledge = self._knowledge_extractor.extract_relationships(combined_rule, planning_horizon)
        augmented_with_relationships = self._augment_formulas(augmented_with_kleene, relationship_knowledge)

        return augmented_with_relationships

    def simplify_single(self, rule: Formula, planning_horizon: int) -> Formula:
        simplified = self.simplify([rule], planning_horizon)
        return simplified[0] if simplified else Formula.true_formula()

    @staticmethod
    def _augment_formulas(formulas: Iterable[Formula], knowledge: KnowledgeSequence) -> List[Formula]:
        augmenter = Augmenter(knowledge)
        return [augmented for formula in formulas if not (augmented := augmenter.augment(formula)).is_true()]
