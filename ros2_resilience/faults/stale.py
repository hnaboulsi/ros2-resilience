"""A stateful fault that holds one input sample while enabled."""

from copy import deepcopy
from typing import cast

_UNSET = object()


class StaleHold[T]:
    """Forward current inputs when disabled and a fixed sample when enabled."""

    def __init__(self) -> None:
        self._enabled = False
        self._latest: T | object = _UNSET
        self._held: T | object = _UNSET

    def set_enabled(self, enabled: bool) -> None:
        """Enable or disable stale output generation."""
        if not isinstance(enabled, bool):
            raise TypeError("enabled must be a bool")

        if enabled and not self._enabled and self._latest is not _UNSET:
            self._held = deepcopy(cast(T, self._latest))
        elif not enabled and self._enabled:
            self._held = _UNSET

        self._enabled = enabled

    def receive(self, message: T) -> T:
        """Record an input and return either it or the enabled stale sample."""
        self._latest = deepcopy(message)

        if not self._enabled:
            return deepcopy(message)

        if self._held is _UNSET:
            self._held = deepcopy(message)

        return deepcopy(cast(T, self._held))
