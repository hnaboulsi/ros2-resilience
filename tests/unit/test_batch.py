from __future__ import annotations

import pytest

from ros2_resilience.core.batch import derive_seed, generate_batch_id, trial_id


def test_derive_seed_is_base_plus_index() -> None:
    assert derive_seed(7, 0) == 7
    assert derive_seed(7, 3) == 10


def test_derive_seed_rejects_negative_index() -> None:
    with pytest.raises(ValueError):
        derive_seed(7, -1)


def test_trial_id_combines_batch_and_index() -> None:
    assert trial_id("abc123", 0) == "abc123-0000"
    assert trial_id("abc123", 42) == "abc123-0042"


def test_trial_id_rejects_negative_index() -> None:
    with pytest.raises(ValueError):
        trial_id("abc123", -1)


def test_generate_batch_id_is_unique_and_nonempty() -> None:
    a, b = generate_batch_id(), generate_batch_id()
    assert a and b
    assert a != b
