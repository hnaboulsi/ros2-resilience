"""Dropout decisions avoid sampling when their result is predetermined."""

import math
from collections.abc import Callable


class Dropout:
    """Decide whether to drop an item according to a fixed probability.

    ``probability`` and values returned by ``random_value`` may be built-in
    ``int`` or ``float`` values, except booleans. Non-numeric values raise
    :class:`TypeError`; non-finite or out-of-range numeric values raise
    :class:`ValueError`.
    """

    def __init__(self, probability: float, *, random_value: Callable[[], float]) -> None:
        self._probability = self._validate_probability(probability)
        if not callable(random_value):
            raise TypeError("random_value must be callable")
        self._random_value = random_value

    def should_drop(self, *, enabled: bool = True) -> bool:
        """Return whether an item should be dropped.

        A disabled dropout and probabilities of zero or one return without
        sampling the injected random source.
        """
        if not isinstance(enabled, bool):
            raise TypeError("enabled must be a bool")
        if not enabled or self._probability == 0.0:
            return False
        if self._probability == 1.0:
            return True

        value = self._random_value()
        self._validate_random_value(value)
        return value < self._probability

    @staticmethod
    def _validate_probability(probability: float | int) -> float:
        if type(probability) not in (float, int):
            raise TypeError("probability must be an int or float")
        if not math.isfinite(probability) or not 0.0 <= probability <= 1.0:
            raise ValueError("probability must be finite and within [0.0, 1.0]")
        return float(probability)

    @staticmethod
    def _validate_random_value(value: float | int) -> None:
        if type(value) not in (float, int):
            raise TypeError("random_value must return an int or float")
        if not math.isfinite(value) or not 0.0 <= value < 1.0:
            raise ValueError("random_value must return a finite value within [0.0, 1.0)")
