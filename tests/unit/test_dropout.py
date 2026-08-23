import math
import random

import pytest

from ros2_resilience.faults.dropout import Dropout


def fail_if_called() -> float:
    raise AssertionError("random source should not have been sampled")


@pytest.mark.parametrize(
    ("probability", "expected"),
    [(0.0, False), (1.0, True), (0, False), (1, True)],
)
def test_extreme_probabilities_do_not_sample_rng(probability: float, expected: bool) -> None:
    assert Dropout(probability, random_value=fail_if_called).should_drop() is expected


def test_disabled_dropout_does_not_sample_partial_rng() -> None:
    assert Dropout(0.5, random_value=fail_if_called).should_drop(enabled=False) is False


def test_partial_probability_uses_strict_boundary() -> None:
    assert Dropout(0.5, random_value=lambda: 0.499_999).should_drop() is True
    assert Dropout(0.5, random_value=lambda: 0.5).should_drop() is False


def test_seeded_random_source_produces_exact_decisions() -> None:
    source = random.Random(7)
    dropout = Dropout(0.5, random_value=source.random)

    assert [dropout.should_drop() for _ in range(5)] == [True, True, False, True, False]


@pytest.mark.parametrize(
    "probability",
    [True, False, "0.5", None],
)
def test_invalid_probability_type_is_rejected(probability: object) -> None:
    with pytest.raises(TypeError, match="probability must be an int or float"):
        Dropout(probability, random_value=lambda: 0.25)  # type: ignore[arg-type]


@pytest.mark.parametrize("probability", [math.nan, math.inf, -math.inf, -0.1, 1.1])
def test_invalid_probability_value_is_rejected(probability: float) -> None:
    with pytest.raises(ValueError, match="probability must be finite"):
        Dropout(probability, random_value=lambda: 0.25)


@pytest.mark.parametrize("value", [True, False, "0.25", None])
def test_invalid_random_result_type_is_rejected(value: object) -> None:
    def random_value() -> object:
        return value

    with pytest.raises(TypeError, match="random_value must return an int or float"):
        Dropout(0.5, random_value=random_value).should_drop()  # type: ignore[arg-type]


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf, -0.1, 1.0])
def test_invalid_random_result_value_is_rejected(value: float) -> None:
    with pytest.raises(ValueError, match="random_value must return a finite"):
        Dropout(0.5, random_value=lambda: value).should_drop()


def test_integer_random_result_is_accepted() -> None:
    assert Dropout(0.5, random_value=lambda: 0).should_drop() is True


@pytest.mark.parametrize("enabled", [0, 1, None, "yes"])
def test_non_boolean_enabled_is_rejected(enabled: object) -> None:
    with pytest.raises(TypeError, match="enabled must be a bool"):
        Dropout(0.5, random_value=fail_if_called).should_drop(  # type: ignore[arg-type]
            enabled=enabled
        )


def test_non_callable_random_source_is_rejected() -> None:
    with pytest.raises(TypeError, match="random_value must be callable"):
        Dropout(0.5, random_value=object())  # type: ignore[arg-type]
