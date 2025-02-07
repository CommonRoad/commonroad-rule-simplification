import abc
from abc import abstractmethod
from typing import List, Optional, Set

from ltl_augmentation import Formula


class TrafficRule(abc.ABC):
    """Abstract base class for traffic rules."""

    @abstractmethod
    def instantiate(self, obstacle_ids: Set[int], start: int = 0, end: Optional[int] = None) -> List[Formula]:
        """Instantiate the rule for a set of obstacles.

        :param obstacle_ids: The IDs of the obstacles.
        :param start: The start time where the rule must begin to hold.
        :param end: The end time after which the rule need not hold anymore. If None, the rule must hold indefinitely.
        :return: The instantiated traffic rules. The rules in the list are to be interpreted as a conjunction.
        """
        pass
