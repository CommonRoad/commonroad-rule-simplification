import abc
from abc import abstractmethod
from typing import Optional, Set

from mltl_simplification import Formula


class TrafficRule(abc.ABC):
    @abstractmethod
    def instantiate(self, obstacle_ids: Set[int], start: int = 0, end: Optional[int] = None) -> Formula:
        pass
