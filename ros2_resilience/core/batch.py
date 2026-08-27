"""Trial identifier and seed bookkeeping for repeated-trial batches."""

from __future__ import annotations

import uuid

__all__ = ["derive_seed", "generate_batch_id", "trial_id"]


def generate_batch_id() -> str:
    """Return a new random batch identifier."""
    return uuid.uuid4().hex


def derive_seed(base_seed: int, trial_index: int) -> int:
    """Return the effective seed for ``trial_index`` (0-based) in a batch."""
    if trial_index < 0:
        raise ValueError("trial_index must be nonnegative")
    return base_seed + trial_index


def trial_id(batch_id: str, trial_index: int) -> str:
    """Return a stable identifier combining the batch id and trial index."""
    if trial_index < 0:
        raise ValueError("trial_index must be nonnegative")
    return f"{batch_id}-{trial_index:04d}"
