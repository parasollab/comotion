#!/usr/bin/env python3
"""Analyze the expanded eight-policy AO-ARC Panda Cage restart sweep.

The analyzer is intentionally stdlib-only.  It validates the exact paired
design (five tasks, ten seeds, and eight policies), then writes deterministic
Markdown, CSV, and JSON artifacts.  Retained trials from before the recursive
history/random-restart telemetry was added are supported: settings are
recovered from the legacy one-hop boolean, counters are derived from bounded
attempt records when possible, and unavailable or inferred values retain an
explicit source label.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping, Sequence


POLICIES = (
    "full_restart",
    "violators_only",
    "violators_only_random_25",
    "violators_only_random_50",
    "history_one_hop",
    "history_two_hop",
    "history_one_hop_random_25",
    "history_two_hop_random_25",
)

POLICY_LABELS = {
    "full_restart": "Full restart",
    "violators_only": "Violators only (depth 0)",
    "violators_only_random_25": "Violators only + random 25%",
    "violators_only_random_50": "Violators only + random 50%",
    "history_one_hop": "History depth 1",
    "history_two_hop": "History depth 2",
    "history_one_hop_random_25": "History depth 1 + random 25%",
    "history_two_hop_random_25": "History depth 2 + random 25%",
}

# selective bounded replanning, repair-history depth, random restart probability
POLICY_SETTINGS = {
    "full_restart": (False, 0, 0.0),
    "violators_only": (True, 0, 0.0),
    "violators_only_random_25": (True, 0, 0.25),
    "violators_only_random_50": (True, 0, 0.50),
    "history_one_hop": (True, 1, 0.0),
    "history_two_hop": (True, 2, 0.0),
    "history_one_hop_random_25": (True, 1, 0.25),
    "history_two_hop_random_25": (True, 2, 0.25),
}

# These three retained policies predate the numeric depth/probability options.
# Their archived records may omit those fields and encode depth through the
# legacy Boolean.  Every subsequently collected policy must record all three
# settings explicitly, which prevents a mislabeled trial from passing solely
# because its directory name implies the expected configuration.
LEGACY_INFERRED_POLICIES = {
    "full_restart",
    "violators_only",
    "history_one_hop",
}

EXPECTED_TASKS = tuple(range(5))
EXPECTED_SEEDS = tuple(range(10))
EXPECTED_TIME_LIMIT_SECONDS = 90.0
EXPECTED_TRIALS = len(EXPECTED_TASKS) * len(EXPECTED_SEEDS) * len(POLICIES)

OUTPUT_STEM = "ao_arc_expanded_restart_policy"


class AnalysisError(RuntimeError):
    """Raised when a result root does not satisfy the experiment contract."""


@dataclass(frozen=True)
class SourcedCount:
    value: int | None
    source: str


@dataclass(frozen=True)
class SourcedFloat:
    value: float | None
    source: str


@dataclass(frozen=True)
class Trial:
    path: Path
    task: int
    seed: int
    policy: str
    initial_makespan: int
    final_makespan: int
    planning_time_seconds: float
    selective_replanning: bool
    history_depth: int
    random_restart_probability: float
    setting_sources: str
    attempts: SourcedCount
    improvements: SourcedCount
    rejections: SourcedCount
    random_full_restarts: SourcedCount
    random_full_restart_improvements: SourcedCount
    paths_replanned: SourcedCount
    paths_reused: SourcedCount
    conflict_pairs_skipped: SourcedCount
    history_expanded_robots: SourcedCount
    conflict_detection_seconds: SourcedFloat

    @property
    def improvement_timesteps(self) -> int:
        return self.initial_makespan - self.final_makespan

    @property
    def improvement_fraction(self) -> float:
        return self.improvement_timesteps / self.initial_makespan


@dataclass(frozen=True)
class Comparison:
    comparison: str
    candidate: str
    reference: str
    category: str


COMPARISONS = (
    Comparison(
        "violators_only_vs_full_restart",
        "violators_only",
        "full_restart",
        "versus_full_restart",
    ),
    Comparison(
        "violators_only_random_25_vs_full_restart",
        "violators_only_random_25",
        "full_restart",
        "versus_full_restart",
    ),
    Comparison(
        "violators_only_random_50_vs_full_restart",
        "violators_only_random_50",
        "full_restart",
        "versus_full_restart",
    ),
    Comparison(
        "history_one_hop_vs_full_restart",
        "history_one_hop",
        "full_restart",
        "versus_full_restart",
    ),
    Comparison(
        "history_two_hop_vs_full_restart",
        "history_two_hop",
        "full_restart",
        "versus_full_restart",
    ),
    Comparison(
        "history_one_hop_random_25_vs_full_restart",
        "history_one_hop_random_25",
        "full_restart",
        "versus_full_restart",
    ),
    Comparison(
        "history_two_hop_random_25_vs_full_restart",
        "history_two_hop_random_25",
        "full_restart",
        "versus_full_restart",
    ),
    Comparison(
        "random_25_vs_random_0_depth_0",
        "violators_only_random_25",
        "violators_only",
        "random_restart_effect",
    ),
    Comparison(
        "random_50_vs_random_0_depth_0",
        "violators_only_random_50",
        "violators_only",
        "random_restart_effect",
    ),
    Comparison(
        "random_50_vs_random_25_depth_0",
        "violators_only_random_50",
        "violators_only_random_25",
        "random_restart_probability_effect",
    ),
    Comparison(
        "history_one_hop_vs_violators_only",
        "history_one_hop",
        "violators_only",
        "depth_ladder",
    ),
    Comparison(
        "history_two_hop_vs_violators_only",
        "history_two_hop",
        "violators_only",
        "depth_ladder",
    ),
    Comparison(
        "history_one_hop_random_25_vs_violators_only_random_25",
        "history_one_hop_random_25",
        "violators_only_random_25",
        "depth_ladder",
    ),
    Comparison(
        "history_two_hop_random_25_vs_violators_only_random_25",
        "history_two_hop_random_25",
        "violators_only_random_25",
        "depth_ladder",
    ),
    Comparison(
        "depth_2_vs_depth_1_random_0",
        "history_two_hop",
        "history_one_hop",
        "depth_effect",
    ),
    Comparison(
        "depth_2_vs_depth_1_random_25",
        "history_two_hop_random_25",
        "history_one_hop_random_25",
        "depth_effect",
    ),
    Comparison(
        "random_25_vs_random_0_depth_1",
        "history_one_hop_random_25",
        "history_one_hop",
        "random_restart_effect",
    ),
    Comparison(
        "random_25_vs_random_0_depth_2",
        "history_two_hop_random_25",
        "history_two_hop",
        "random_restart_effect",
    ),
)

# Each stochastic policy is compared only with the zero-random policy at the
# same repair-history depth.  Probability is part of the key so strata can be
# pooled across depths without ever conflating the 25% and 50% treatments.
RANDOM_EFFECT_POLICY_PAIRS = {
    (0.25, 0): ("violators_only_random_25", "violators_only"),
    (0.25, 1): ("history_one_hop_random_25", "history_one_hop"),
    (0.25, 2): ("history_two_hop_random_25", "history_two_hop"),
    (0.50, 0): ("violators_only_random_50", "violators_only"),
}


COUNTER_FIELDS = (
    "attempts",
    "improvements",
    "rejections",
    "random_full_restarts",
    "random_full_restart_improvements",
    "paths_replanned",
    "paths_reused",
    "conflict_pairs_skipped",
    "history_expanded_robots",
)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Validate and analyze the expanded eight-policy AO-ARC restart "
            "experiment on Panda Cage n=4."
        )
    )
    parser.add_argument(
        "--root",
        type=Path,
        required=True,
        help="Sweep result root containing trials/**/trial.json.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help=(
            "Artifact directory (default: "
            "ROOT/expanded_restart_policy_analysis)."
        ),
    )
    return parser.parse_args(argv)


def read_json(path: Path) -> dict[str, object]:
    try:
        with path.open(encoding="utf-8") as handle:
            value = json.load(handle)
    except (OSError, json.JSONDecodeError) as exc:
        raise AnalysisError(f"Unable to read {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise AnalysisError(f"Expected a JSON object in {path}")
    return value


def object_value(value: object, field: str, path: Path) -> dict[str, object]:
    if not isinstance(value, dict):
        raise AnalysisError(f"Expected object {field} in {path}")
    return value


def optional_object(value: object) -> dict[str, object]:
    return value if isinstance(value, dict) else {}


def nonnegative_int(value: object, field: str, path: Path) -> int:
    if isinstance(value, bool):
        raise AnalysisError(f"Invalid integer {field}={value!r} in {path}")
    try:
        result = int(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise AnalysisError(f"Invalid integer {field}={value!r} in {path}") from exc
    if result < 0 or isinstance(value, float) and not value.is_integer():
        raise AnalysisError(f"Invalid nonnegative integer {field}={value!r} in {path}")
    return result


def positive_int(value: object, field: str, path: Path) -> int:
    result = nonnegative_int(value, field, path)
    if result == 0:
        raise AnalysisError(f"Expected positive integer {field} in {path}")
    return result


def finite_float(value: object, field: str, path: Path) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise AnalysisError(f"Invalid number {field}={value!r} in {path}") from exc
    if not math.isfinite(result):
        raise AnalysisError(f"Non-finite number {field}={value!r} in {path}")
    return result


def optional_nonnegative_int(
    mapping: Mapping[str, object], key: str, path: Path
) -> int | None:
    if key not in mapping or mapping[key] is None:
        return None
    return nonnegative_int(mapping[key], key, path)


def optional_nonnegative_float(
    mapping: Mapping[str, object], key: str, path: Path
) -> float | None:
    if key not in mapping or mapping[key] is None:
        return None
    result = finite_float(mapping[key], key, path)
    if result < 0.0:
        raise AnalysisError(f"Expected nonnegative {key} in {path}")
    return result


def explicit_bool(value: object, field: str, path: Path) -> bool:
    if not isinstance(value, bool):
        raise AnalysisError(f"Expected Boolean {field} in {path}, found {value!r}")
    return value


def command_option(command: object, option: str) -> str | None:
    if not isinstance(command, list):
        return None
    words = [str(word) for word in command]
    try:
        index = words.index(option)
    except ValueError:
        return None
    if index + 1 >= len(words):
        return None
    return words[index + 1]


def command_switch(command: object, positive: str, negative: str) -> bool | None:
    if not isinstance(command, list):
        return None
    words = {str(word) for word in command}
    has_positive = positive in words
    has_negative = negative in words
    if has_positive and has_negative:
        return None
    if has_positive:
        return True
    if has_negative:
        return False
    return None


def reconcile_setting(
    values: Sequence[tuple[str, object]],
    *,
    expected: object,
    field: str,
    path: Path,
) -> tuple[object, str]:
    present = [(source, value) for source, value in values if value is not None]
    if present:
        first_value = present[0][1]
        if any(value != first_value for _source, value in present[1:]):
            raise AnalysisError(
                f"Conflicting {field} settings in {path}: "
                + ", ".join(f"{source}={value!r}" for source, value in present)
            )
        if first_value != expected:
            raise AnalysisError(
                f"Policy setting mismatch in {path}: {field}={first_value!r}, "
                f"expected {expected!r}"
            )
        return first_value, "+".join(source for source, _value in present)
    return expected, "policy_name_fallback"


def parse_settings(
    trial: Mapping[str, object],
    stats: Mapping[str, object],
    params: Mapping[str, object],
    policy: str,
    path: Path,
) -> tuple[bool, int, float, str]:
    expected_selective, expected_depth, expected_probability = POLICY_SETTINGS[policy]
    command = trial.get("command")

    stats_selective = None
    if "selective_bounded_replanning" in stats:
        stats_selective = explicit_bool(
            stats["selective_bounded_replanning"],
            "selective_bounded_replanning",
            path,
        )
    params_selective = None
    if "ao_arc_selective_replanning" in params:
        params_selective = explicit_bool(
            params["ao_arc_selective_replanning"],
            "ao_arc_selective_replanning",
            path,
        )
    command_selective = command_switch(
        command,
        "--ao-arc-selective-replanning",
        "--no-ao-arc-selective-replanning",
    )
    selective, selective_source = reconcile_setting(
        (
            ("planner_stats", stats_selective),
            ("params", params_selective),
            ("command", command_selective),
        ),
        expected=expected_selective,
        field="selective bounded replanning",
        path=path,
    )
    if policy not in LEGACY_INFERRED_POLICIES and not any(
        value is not None
        for value in (stats_selective, params_selective, command_selective)
    ):
        raise AnalysisError(
            f"Missing explicit selective bounded replanning setting in {path}"
        )

    depth_values: list[tuple[str, object]] = []
    if "repair_history_replanning_depth" in stats:
        depth_values.append(
            (
                "planner_stats",
                nonnegative_int(
                    stats["repair_history_replanning_depth"],
                    "repair_history_replanning_depth",
                    path,
                ),
            )
        )
    if "ao_arc_repair_history_replanning_depth" in params:
        depth_values.append(
            (
                "params",
                nonnegative_int(
                    params["ao_arc_repair_history_replanning_depth"],
                    "ao_arc_repair_history_replanning_depth",
                    path,
                ),
            )
        )
    command_depth = command_option(
        command, "--ao-arc-repair-history-replanning-depth"
    )
    if command_depth is not None:
        depth_values.append(
            (
                "command",
                nonnegative_int(
                    command_depth,
                    "--ao-arc-repair-history-replanning-depth",
                    path,
                ),
            )
        )

    # Legacy records used a Boolean that meant exactly depth one.
    legacy_expansion_values: list[tuple[str, bool]] = []
    if "expand_replanning_from_repair_history" in stats:
        legacy_expansion_values.append(
            (
                "planner_stats_legacy_bool",
                explicit_bool(
                    stats["expand_replanning_from_repair_history"],
                    "expand_replanning_from_repair_history",
                    path,
                ),
            )
        )
    if "ao_arc_expand_replanning_from_repair_history" in params:
        legacy_expansion_values.append(
            (
                "params_legacy_bool",
                explicit_bool(
                    params["ao_arc_expand_replanning_from_repair_history"],
                    "ao_arc_expand_replanning_from_repair_history",
                    path,
                ),
            )
        )
    command_expansion = command_switch(
        command,
        "--ao-arc-expand-replanning-from-repair-history",
        "--no-ao-arc-expand-replanning-from-repair-history",
    )
    if command_expansion is not None:
        legacy_expansion_values.append(("command_legacy_bool", command_expansion))
    if depth_values:
        for source, enabled in legacy_expansion_values:
            explicit_depth = depth_values[0][1]
            if enabled != (explicit_depth != 0):
                raise AnalysisError(
                    f"Conflicting history depth and {source} in {path}"
                )
    else:
        depth_values.extend(
            (source, 1 if enabled else 0)
            for source, enabled in legacy_expansion_values
        )
    if policy not in LEGACY_INFERRED_POLICIES and not depth_values:
        raise AnalysisError(
            f"Missing explicit repair-history replanning depth in {path}"
        )
    depth, depth_source = reconcile_setting(
        depth_values,
        expected=expected_depth,
        field="repair-history replanning depth",
        path=path,
    )

    probability_values: list[tuple[str, object]] = []
    if "random_full_restart_probability" in stats:
        probability_values.append(
            (
                "planner_stats",
                finite_float(
                    stats["random_full_restart_probability"],
                    "random_full_restart_probability",
                    path,
                ),
            )
        )
    if "ao_arc_random_full_restart_probability" in params:
        probability_values.append(
            (
                "params",
                finite_float(
                    params["ao_arc_random_full_restart_probability"],
                    "ao_arc_random_full_restart_probability",
                    path,
                ),
            )
        )
    command_probability = command_option(
        command, "--ao-arc-random-full-restart-probability"
    )
    if command_probability is not None:
        probability_values.append(
            (
                "command",
                finite_float(
                    command_probability,
                    "--ao-arc-random-full-restart-probability",
                    path,
                ),
            )
        )
    if policy not in LEGACY_INFERRED_POLICIES and not probability_values:
        raise AnalysisError(
            f"Missing explicit random full-restart probability in {path}"
        )
    if probability_values:
        if any(
            not math.isclose(
                float(value), expected_probability, rel_tol=0.0, abs_tol=1e-12
            )
            for _source, value in probability_values
        ):
            raise AnalysisError(
                f"Policy random-restart probability mismatch in {path}: "
                + ", ".join(
                    f"{source}={value!r}" for source, value in probability_values
                )
                + f", expected {expected_probability}"
            )
        probability = expected_probability
        probability_source = "+".join(
            source for source, _value in probability_values
        )
    else:
        probability = expected_probability
        probability_source = "policy_name_fallback"

    return (
        bool(selective),
        int(depth),
        float(probability),
        ";".join(
            (
                f"selective:{selective_source}",
                f"depth:{depth_source}",
                f"probability:{probability_source}",
            )
        ),
    )


def parse_attempts(stats: Mapping[str, object], path: Path) -> list[dict[str, object]] | None:
    if "bounded_attempts" not in stats:
        return None
    raw_attempts = stats["bounded_attempts"]
    if not isinstance(raw_attempts, list):
        raise AnalysisError(f"Expected planner_stats.bounded_attempts list in {path}")
    attempts: list[dict[str, object]] = []
    for index, raw_attempt in enumerate(raw_attempts):
        if not isinstance(raw_attempt, dict):
            raise AnalysisError(f"Invalid bounded attempt {index} in {path}")
        attempts.append(raw_attempt)
    return attempts


def sum_attempt_int(
    attempts: Sequence[Mapping[str, object]] | None,
    key: str,
    path: Path,
) -> int | None:
    if attempts is None or any(key not in attempt for attempt in attempts):
        return None
    return sum(nonnegative_int(attempt[key], f"bounded_attempts[].{key}", path) for attempt in attempts)


def sum_attempt_bool(
    attempts: Sequence[Mapping[str, object]] | None,
    key: str,
    path: Path,
) -> int | None:
    if attempts is None or any(key not in attempt for attempt in attempts):
        return None
    return sum(
        explicit_bool(attempt[key], f"bounded_attempts[].{key}", path)
        for attempt in attempts
    )


def sum_attempt_bool_conjunction(
    attempts: Sequence[Mapping[str, object]] | None,
    first_key: str,
    second_key: str,
    path: Path,
) -> int | None:
    if attempts is None or any(
        first_key not in attempt or second_key not in attempt
        for attempt in attempts
    ):
        return None
    return sum(
        explicit_bool(
            attempt[first_key], f"bounded_attempts[].{first_key}", path
        )
        and explicit_bool(
            attempt[second_key], f"bounded_attempts[].{second_key}", path
        )
        for attempt in attempts
    )


def sum_attempt_float(
    attempts: Sequence[Mapping[str, object]] | None,
    key: str,
    path: Path,
) -> float | None:
    if attempts is None or any(key not in attempt for attempt in attempts):
        return None
    values = [
        finite_float(attempt[key], f"bounded_attempts[].{key}", path)
        for attempt in attempts
    ]
    if any(value < 0.0 for value in values):
        raise AnalysisError(f"Negative bounded_attempts[].{key} in {path}")
    return math.fsum(values)


def reconcile_count(
    recorded: int | None,
    derived: int | None,
    *,
    field: str,
    path: Path,
) -> SourcedCount:
    if recorded is not None and derived is not None and recorded != derived:
        raise AnalysisError(
            f"Recorded/derived {field} mismatch in {path}: "
            f"{recorded} != {derived}"
        )
    if recorded is not None and derived is not None:
        return SourcedCount(recorded, "planner_stats+bounded_attempts")
    if recorded is not None:
        return SourcedCount(recorded, "planner_stats")
    if derived is not None:
        return SourcedCount(derived, "bounded_attempts")
    return SourcedCount(None, "unavailable")


def parse_trial(path: Path) -> Trial:
    trial = read_json(path)
    row = object_value(trial.get("result_row"), "result_row", path)
    metrics = object_value(trial.get("metrics"), "metrics", path)
    stats = optional_object(metrics.get("planner_stats"))
    params = optional_object(trial.get("params"))
    context = optional_object(metrics.get("benchmark_context"))

    policy = str(row.get("param_set", ""))
    if policy not in POLICIES:
        raise AnalysisError(f"Unexpected policy {policy!r} in {path}")
    if trial.get("status") != "complete":
        raise AnalysisError(f"Trial is not complete: {path}")
    if trial.get("returncode") != 0 or row.get("returncode") != 0:
        raise AnalysisError(f"Trial did not return zero: {path}")
    if trial.get("timed_out") is not False or row.get("timed_out") is not False:
        raise AnalysisError(f"Trial hit its external timeout: {path}")
    if row.get("success") is not True or metrics.get("success") is not True:
        raise AnalysisError(f"Trial has no successful incumbent: {path}")
    if str(row.get("method", "")) != "ao_arc":
        raise AnalysisError(f"Expected AO-ARC method in {path}")
    planner_status = str(metrics.get("planner_status", "")).lower()
    if "exact" not in planner_status:
        raise AnalysisError(f"Expected exact planner status in {path}")

    app = str(row.get("app", ""))
    app_match = re.fullmatch(r"panda_cage_n4_task([0-4])", app)
    if not app_match:
        raise AnalysisError(f"Unexpected Panda Cage app {app!r} in {path}")
    app_task = int(app_match.group(1))
    context_task = context.get("task_index")
    task = app_task if context_task is None else nonnegative_int(context_task, "task_index", path)
    if task != app_task:
        raise AnalysisError(f"App/context task mismatch in {path}")
    seed = nonnegative_int(row.get("seed"), "seed", path)
    if "num_robots" not in context:
        raise AnalysisError(f"Missing benchmark_context.num_robots in {path}")
    if nonnegative_int(context["num_robots"], "num_robots", path) != 4:
        raise AnalysisError(f"Expected four robots in {path}")
    if "time_limit_seconds" not in context:
        raise AnalysisError(
            f"Missing benchmark_context.time_limit_seconds in {path}"
        )
    time_limit = finite_float(
        context["time_limit_seconds"], "time_limit_seconds", path
    )
    if not math.isclose(
        time_limit,
        EXPECTED_TIME_LIMIT_SECONDS,
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise AnalysisError(f"Expected a 90-second planning limit in {path}")

    events_value = stats.get("solution_events")
    if not isinstance(events_value, list) or not events_value:
        raise AnalysisError(f"Missing AO-ARC solution_events in {path}")
    event_makespans: list[int] = []
    event_times: list[float] = []
    for index, event_value in enumerate(events_value):
        if not isinstance(event_value, dict):
            raise AnalysisError(f"Invalid solution event {index} in {path}")
        event_makespans.append(
            positive_int(
                event_value.get("makespan_timesteps"),
                f"solution_events[{index}].makespan_timesteps",
                path,
            )
        )
        event_times.append(
            finite_float(
                event_value.get("elapsed_seconds"),
                f"solution_events[{index}].elapsed_seconds",
                path,
            )
        )
    if event_times[0] < 0.0 or any(
        later <= earlier for earlier, later in zip(event_times, event_times[1:])
    ):
        raise AnalysisError(f"Solution event times are not strictly increasing in {path}")
    if any(
        later >= earlier
        for earlier, later in zip(event_makespans, event_makespans[1:])
    ):
        raise AnalysisError(f"Solution makespans are not strictly decreasing in {path}")

    initial_makespan = event_makespans[0]
    final_makespan = event_makespans[-1]
    metrics_final = positive_int(
        metrics.get("makespan_timesteps"), "makespan_timesteps", path
    )
    if final_makespan != metrics_final:
        raise AnalysisError(f"Final AO event/metric makespan mismatch in {path}")
    planning_time = finite_float(
        metrics.get("planning_time_seconds"), "planning_time_seconds", path
    )
    if planning_time < event_times[-1]:
        raise AnalysisError(f"Final solution event is after planning time in {path}")

    selective, depth, probability, setting_sources = parse_settings(
        trial, stats, params, policy, path
    )
    if "selective_initial_conflict_scan" in stats and not explicit_bool(
        stats["selective_initial_conflict_scan"],
        "selective_initial_conflict_scan",
        path,
    ):
        raise AnalysisError(f"Selective initial conflict scan is disabled in {path}")

    attempts_list = parse_attempts(stats, path)
    recorded_attempts = optional_nonnegative_int(stats, "num_bounded_attempts", path)
    derived_attempts = None if attempts_list is None else len(attempts_list)
    attempts = reconcile_count(
        recorded_attempts,
        derived_attempts,
        field="bounded attempts",
        path=path,
    )

    recorded_improvements = optional_nonnegative_int(stats, "num_improvements", path)
    derived_improvements = len(events_value) - 1
    improvements = reconcile_count(
        recorded_improvements,
        derived_improvements,
        field="improvements",
        path=path,
    )
    recorded_rejections = optional_nonnegative_int(stats, "num_rejections", path)
    derived_rejections = None
    if attempts.value is not None and improvements.value is not None:
        if improvements.value > attempts.value:
            raise AnalysisError(f"More improvements than bounded attempts in {path}")
        derived_rejections = attempts.value - improvements.value
    rejections = reconcile_count(
        recorded_rejections,
        derived_rejections,
        field="rejections",
        path=path,
    )

    random_full_restarts = reconcile_count(
        optional_nonnegative_int(stats, "num_random_full_restarts", path),
        sum_attempt_bool(attempts_list, "random_full_restart", path),
        field="random full restarts",
        path=path,
    )
    if random_full_restarts.value is None and math.isclose(
        probability, 0.0, rel_tol=0.0, abs_tol=1e-12
    ):
        # A zero configured probability proves that no stochastic full restart
        # occurred even in records that predate the explicit telemetry field.
        random_full_restarts = SourcedCount(0, "inferred_zero_probability")
    if (
        random_full_restarts.value is not None
        and attempts.value is not None
        and random_full_restarts.value > attempts.value
    ):
        raise AnalysisError(f"More random restarts than bounded attempts in {path}")

    random_full_restart_improvements = reconcile_count(
        optional_nonnegative_int(
            stats, "num_random_full_restart_improvements", path
        ),
        sum_attempt_bool_conjunction(
            attempts_list, "random_full_restart", "improved", path
        ),
        field="random full-restart improvements",
        path=path,
    )
    if (
        random_full_restart_improvements.value is None
        and random_full_restarts.value == 0
    ):
        # This is exact even for legacy p=0 records: no realized stochastic
        # restart implies that none of the improvements came from one.
        random_full_restart_improvements = SourcedCount(
            0, "inferred_zero_random_restarts"
        )
    if (
        random_full_restart_improvements.value is not None
        and random_full_restarts.value is not None
        and random_full_restart_improvements.value > random_full_restarts.value
    ):
        raise AnalysisError(
            f"More random-restart improvements than random restarts in {path}"
        )

    paths_replanned = reconcile_count(
        optional_nonnegative_int(stats, "total_paths_replanned", path),
        sum_attempt_int(attempts_list, "num_paths_replanned", path),
        field="paths replanned",
        path=path,
    )
    paths_reused = reconcile_count(
        optional_nonnegative_int(stats, "total_paths_reused", path),
        sum_attempt_int(attempts_list, "num_paths_reused", path),
        field="paths reused",
        path=path,
    )
    conflict_pairs_skipped = reconcile_count(
        optional_nonnegative_int(
            stats, "total_initial_conflict_pairs_skipped", path
        ),
        sum_attempt_int(attempts_list, "num_conflict_pairs_skipped", path),
        field="initial conflict pairs skipped",
        path=path,
    )
    history_recorded = optional_nonnegative_int(
        stats, "total_history_expanded_replanning_robots", path
    )
    history_alias = optional_nonnegative_int(
        stats, "total_repair_history_expanded_paths", path
    )
    if history_recorded is None:
        history_recorded = history_alias
    elif history_alias is not None and history_alias != history_recorded:
        raise AnalysisError(f"History-expansion total aliases disagree in {path}")
    history_expanded = reconcile_count(
        history_recorded,
        sum_attempt_int(
            attempts_list, "num_repair_history_expanded_robots", path
        ),
        field="history-expanded robot selections",
        path=path,
    )

    conflict_time_recorded = optional_nonnegative_float(
        stats, "total_bounded_conflict_detection_time_seconds", path
    )
    conflict_time_derived = sum_attempt_float(
        attempts_list, "conflict_detection_time_seconds", path
    )
    if (
        conflict_time_recorded is not None
        and conflict_time_derived is not None
        and not math.isclose(
            conflict_time_recorded,
            conflict_time_derived,
            rel_tol=1e-9,
            abs_tol=1e-12,
        )
    ):
        raise AnalysisError(f"Recorded/derived conflict detection time mismatch in {path}")
    if conflict_time_recorded is not None and conflict_time_derived is not None:
        conflict_detection_seconds = SourcedFloat(
            conflict_time_recorded, "planner_stats+bounded_attempts"
        )
    elif conflict_time_recorded is not None:
        conflict_detection_seconds = SourcedFloat(
            conflict_time_recorded, "planner_stats"
        )
    elif conflict_time_derived is not None:
        conflict_detection_seconds = SourcedFloat(
            conflict_time_derived, "bounded_attempts"
        )
    else:
        conflict_detection_seconds = SourcedFloat(None, "unavailable")

    return Trial(
        path=path,
        task=task,
        seed=seed,
        policy=policy,
        initial_makespan=initial_makespan,
        final_makespan=final_makespan,
        planning_time_seconds=planning_time,
        selective_replanning=selective,
        history_depth=depth,
        random_restart_probability=probability,
        setting_sources=setting_sources,
        attempts=attempts,
        improvements=improvements,
        rejections=rejections,
        random_full_restarts=random_full_restarts,
        random_full_restart_improvements=random_full_restart_improvements,
        paths_replanned=paths_replanned,
        paths_reused=paths_reused,
        conflict_pairs_skipped=conflict_pairs_skipped,
        history_expanded_robots=history_expanded,
        conflict_detection_seconds=conflict_detection_seconds,
    )


def load_trials(root: Path) -> list[Trial]:
    trial_paths = sorted((root / "trials").rglob("trial.json"))
    if not trial_paths:
        raise AnalysisError(f"No trial.json files found below {root / 'trials'}")
    trials = [parse_trial(path) for path in trial_paths]

    expected_identities = {
        (task, seed, policy)
        for task in EXPECTED_TASKS
        for seed in EXPECTED_SEEDS
        for policy in POLICIES
    }
    actual_identities: set[tuple[int, int, str]] = set()
    duplicates: list[tuple[int, int, str]] = []
    for trial in trials:
        identity = (trial.task, trial.seed, trial.policy)
        if identity in actual_identities:
            duplicates.append(identity)
        actual_identities.add(identity)
    if duplicates:
        raise AnalysisError(f"Duplicate trial identities: {sorted(duplicates)}")
    if actual_identities != expected_identities or len(trials) != EXPECTED_TRIALS:
        missing = sorted(expected_identities - actual_identities)
        extra = sorted(actual_identities - expected_identities)
        raise AnalysisError(
            f"Expected exactly {EXPECTED_TRIALS} trials: tasks 0-4 x seeds 0-9 "
            f"x eight policies; found {len(trials)}. Missing={missing}; extra={extra}"
        )

    blocks = group_blocks(trials)
    for block, policy_trials in blocks.items():
        if tuple(policy_trials) != POLICIES:
            raise AnalysisError(f"Policy ordering/coverage mismatch in block {block}")
        initial_values = {trial.initial_makespan for trial in policy_trials.values()}
        if len(initial_values) != 1:
            raise AnalysisError(
                f"Policies have different initial makespans in task/seed {block}: "
                f"{sorted(initial_values)}"
            )
    return sorted(trials, key=lambda trial: (trial.task, trial.seed, POLICIES.index(trial.policy)))


def group_blocks(trials: Sequence[Trial]) -> dict[tuple[int, int], dict[str, Trial]]:
    blocks: dict[tuple[int, int], dict[str, Trial]] = {}
    for trial in trials:
        blocks.setdefault((trial.task, trial.seed), {})[trial.policy] = trial
    return {
        block: {policy: policy_trials[policy] for policy in POLICIES if policy in policy_trials}
        for block, policy_trials in sorted(blocks.items())
    }


def source_category(source: str) -> str:
    if source == "unavailable":
        return "unavailable"
    if source.startswith("inferred_"):
        return "inferred"
    if "bounded_attempts" in source and "planner_stats" not in source:
        return "derived"
    return "recorded"


def availability(values: Sequence[SourcedCount | SourcedFloat]) -> dict[str, int]:
    result = {"recorded": 0, "derived": 0, "inferred": 0, "unavailable": 0}
    for value in values:
        result[source_category(value.source)] += 1
    result["available"] = len(values) - result["unavailable"]
    result["total_trials"] = len(values)
    return result


def optional_total(values: Sequence[SourcedCount]) -> tuple[int | None, int]:
    present = [value.value for value in values if value.value is not None]
    return (sum(present) if present else None, len(present))


def optional_float_total(values: Sequence[SourcedFloat]) -> tuple[float | None, int]:
    present = [value.value for value in values if value.value is not None]
    return (math.fsum(present) if present else None, len(present))


def policy_summary(
    policy_trials: Sequence[Trial],
    blocks: Mapping[tuple[int, int], Mapping[str, Trial]],
) -> dict[str, object]:
    policy = policy_trials[0].policy
    initial = [trial.initial_makespan for trial in policy_trials]
    final = [trial.final_makespan for trial in policy_trials]
    ratios_to_best = []
    for trial in policy_trials:
        block = blocks[(trial.task, trial.seed)]
        best = min(candidate.final_makespan for candidate in block.values())
        ratios_to_best.append(trial.final_makespan / best)

    counter_totals: dict[str, object] = {}
    counter_availability: dict[str, object] = {}
    for field in COUNTER_FIELDS:
        values = [getattr(trial, field) for trial in policy_trials]
        total, available_count = optional_total(values)
        counter_totals[f"{field}_total"] = total
        counter_totals[f"{field}_trials_available"] = available_count
        counter_availability[field] = availability(values)
    conflict_time_total, conflict_time_available = optional_float_total(
        [trial.conflict_detection_seconds for trial in policy_trials]
    )
    counter_totals["conflict_detection_seconds_total"] = conflict_time_total
    counter_totals["conflict_detection_seconds_trials_available"] = (
        conflict_time_available
    )
    counter_availability["conflict_detection_seconds"] = availability(
        [trial.conflict_detection_seconds for trial in policy_trials]
    )

    attempts_total = counter_totals["attempts_total"]
    improvements_total = counter_totals["improvements_total"]
    random_total = counter_totals["random_full_restarts_total"]
    random_rate_trials = [
        trial
        for trial in policy_trials
        if trial.attempts.value is not None
        and trial.random_full_restarts.value is not None
    ]
    random_numerator = sum(
        trial.random_full_restarts.value for trial in random_rate_trials
    )
    random_denominator = sum(
        trial.attempts.value
        for trial in random_rate_trials
    )
    improvement_rate_trials = [
        trial
        for trial in policy_trials
        if trial.attempts.value is not None and trial.improvements.value is not None
    ]
    improvement_numerator = sum(
        trial.improvements.value for trial in improvement_rate_trials
    )
    improvement_denominator = sum(
        trial.attempts.value
        for trial in improvement_rate_trials
    )
    random_improvement_denominator = sum(
        trial.random_full_restarts.value
        for trial in policy_trials
        if trial.random_full_restarts.value is not None
        and trial.random_full_restart_improvements.value is not None
    )
    random_improvement_numerator = sum(
        trial.random_full_restart_improvements.value
        for trial in policy_trials
        if trial.random_full_restarts.value is not None
        and trial.random_full_restart_improvements.value is not None
    )

    return {
        "policy": policy,
        "label": POLICY_LABELS[policy],
        "selective_replanning": policy_trials[0].selective_replanning,
        "history_depth": policy_trials[0].history_depth,
        "configured_random_restart_probability": policy_trials[0].random_restart_probability,
        "trial_count": len(policy_trials),
        "mean_initial_makespan_timesteps": statistics.fmean(initial),
        "median_initial_makespan_timesteps": statistics.median(initial),
        "mean_final_makespan_timesteps": statistics.fmean(final),
        "median_final_makespan_timesteps": statistics.median(final),
        "mean_initial_to_final_improvement_fraction": statistics.fmean(
            trial.improvement_fraction for trial in policy_trials
        ),
        "median_final_over_initial": statistics.median(
            trial.final_makespan / trial.initial_makespan
            for trial in policy_trials
        ),
        "mean_ratio_to_matched_best": statistics.fmean(ratios_to_best),
        "median_ratio_to_matched_best": statistics.median(ratios_to_best),
        "best_or_tied_count": sum(
            math.isclose(ratio, 1.0, rel_tol=0.0, abs_tol=1e-12)
            for ratio in ratios_to_best
        ),
        **counter_totals,
        "improvements_per_bounded_attempt": (
            float(improvement_numerator) / improvement_denominator
            if improvement_denominator
            else None
        ),
        "random_restart_decision_calls": random_denominator,
        "realized_random_restart_rate": (
            float(random_numerator) / random_denominator
            if random_denominator
            else None
        ),
        "random_restart_improvements_for_rate": random_improvement_numerator,
        "random_restart_improvement_denominator": random_improvement_denominator,
        "accepted_improvement_rate_per_random_restart": (
            random_improvement_numerator / random_improvement_denominator
            if random_improvement_denominator
            else None
        ),
        "telemetry_availability": counter_availability,
    }


def policy_summary_by_scope(
    trials: Sequence[Trial],
    blocks: Mapping[tuple[int, int], Mapping[str, Trial]],
) -> tuple[dict[str, dict[str, object]], dict[str, dict[str, dict[str, object]]]]:
    overall = {
        policy: policy_summary(
            [trial for trial in trials if trial.policy == policy], blocks
        )
        for policy in POLICIES
    }
    by_task: dict[str, dict[str, dict[str, object]]] = {}
    for task in EXPECTED_TASKS:
        by_task[str(task)] = {
            policy: policy_summary(
                [
                    trial
                    for trial in trials
                    if trial.task == task and trial.policy == policy
                ],
                blocks,
            )
            for policy in POLICIES
        }
    return overall, by_task


def comparison_row(
    spec: Comparison,
    selected_blocks: Sequence[tuple[tuple[int, int], Mapping[str, Trial]]],
    scope: str,
) -> dict[str, object]:
    deltas: list[int] = []
    relative_deltas: list[float] = []
    for _block_key, block in selected_blocks:
        candidate = block[spec.candidate].final_makespan
        reference = block[spec.reference].final_makespan
        deltas.append(candidate - reference)
        relative_deltas.append((candidate - reference) / reference)
    return {
        "scope": scope,
        "category": spec.category,
        "comparison": spec.comparison,
        "candidate": spec.candidate,
        "reference": spec.reference,
        "pair_count": len(deltas),
        "wins": sum(delta < 0 for delta in deltas),
        "ties": sum(delta == 0 for delta in deltas),
        "losses": sum(delta > 0 for delta in deltas),
        "sum_delta_timesteps": sum(deltas),
        "mean_delta_timesteps": statistics.fmean(deltas),
        "median_delta_timesteps": statistics.median(deltas),
        "mean_relative_delta_fraction": statistics.fmean(relative_deltas),
        "median_relative_delta_fraction": statistics.median(relative_deltas),
        "minimum_delta_timesteps": min(deltas),
        "maximum_delta_timesteps": max(deltas),
    }


def comparison_rows(
    blocks: Mapping[tuple[int, int], Mapping[str, Trial]],
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    all_blocks = list(blocks.items())
    scopes: list[
        tuple[str, list[tuple[tuple[int, int], Mapping[str, Trial]]]]
    ] = [("all_tasks", all_blocks)]
    scopes.extend(
        (
            f"task_{task}",
            [(key, block) for key, block in all_blocks if key[0] == task],
        )
        for task in EXPECTED_TASKS
    )
    for scope, selected in scopes:
        for spec in COMPARISONS:
            rows.append(comparison_row(spec, selected, scope))
    return rows


def random_effect_observations(
    blocks: Mapping[tuple[int, int], Mapping[str, Trial]],
) -> list[dict[str, object]]:
    observations: list[dict[str, object]] = []
    for (probability, depth), (candidate_policy, reference_policy) in (
        RANDOM_EFFECT_POLICY_PAIRS.items()
    ):
        for (task, seed), block in blocks.items():
            candidate = block[candidate_policy]
            reference = block[reference_policy]
            observations.append(
                {
                    "configured_random_restart_probability": probability,
                    "depth": depth,
                    "task": task,
                    "seed": seed,
                    "candidate": candidate_policy,
                    "reference": reference_policy,
                    "delta_timesteps": (
                        candidate.final_makespan - reference.final_makespan
                    ),
                    "reference_final_makespan": reference.final_makespan,
                    "random_full_restarts": candidate.random_full_restarts.value,
                    "random_full_restart_improvements": (
                        candidate.random_full_restart_improvements.value
                    ),
                }
            )
    return observations


def random_stratum_membership(
    observation: Mapping[str, object], family: str, stratum: str
) -> bool | None:
    random_restarts = observation["random_full_restarts"]
    random_improvements = observation["random_full_restart_improvements"]
    if family == "unstratified":
        return True
    if family == "realized_restart_count":
        if random_restarts is None:
            return None
        if stratum == "none":
            return int(random_restarts) == 0
        if stratum == "one_or_more":
            return int(random_restarts) >= 1
    elif family == "accepted_restart_improvement_count":
        if random_improvements is None:
            return None
        if stratum == "zero":
            return int(random_improvements) == 0
        if stratum == "one_or_more":
            return int(random_improvements) >= 1
    elif family == "joint_restart_outcome":
        if random_restarts is None or random_improvements is None:
            return None
        restart_count = int(random_restarts)
        improvement_count = int(random_improvements)
        if stratum == "no_fire":
            return restart_count == 0
        if stratum == "fired_no_accepted_improvement":
            return restart_count >= 1 and improvement_count == 0
        if stratum == "fired_with_accepted_improvement":
            return restart_count >= 1 and improvement_count >= 1
    raise AssertionError(f"Unknown random-effect stratum: {family}/{stratum}")


def summarize_random_stratum(
    observations: Sequence[Mapping[str, object]],
    *,
    scope: str,
    probability: float,
    depth: int | None,
    family: str,
    stratum: str,
) -> dict[str, object]:
    memberships = [
        random_stratum_membership(observation, family, stratum)
        for observation in observations
    ]
    eligible = [
        observation
        for observation, membership in zip(observations, memberships)
        if membership is not None
    ]
    selected = [
        observation
        for observation, membership in zip(observations, memberships)
        if membership is True
    ]
    deltas = [int(observation["delta_timesteps"]) for observation in selected]
    relative_deltas = [
        int(observation["delta_timesteps"])
        / int(observation["reference_final_makespan"])
        for observation in selected
    ]
    probability_percent = int(round(100.0 * probability))
    if depth is None:
        candidate = f"random_{probability_percent}_at_matching_depth"
        reference = "random_0_at_matching_depth"
    else:
        candidate, reference = RANDOM_EFFECT_POLICY_PAIRS[(probability, depth)]
    return {
        "scope": scope,
        "configured_random_restart_probability": probability,
        "depth": "all" if depth is None else depth,
        "candidate": candidate,
        "reference": reference,
        "stratum_family": family,
        "stratum": stratum,
        "scope_pair_count": len(observations),
        "telemetry_eligible_pair_count": len(eligible),
        "telemetry_unavailable_pair_count": len(observations) - len(eligible),
        "pair_count": len(selected),
        "wins": sum(delta < 0 for delta in deltas),
        "ties": sum(delta == 0 for delta in deltas),
        "losses": sum(delta > 0 for delta in deltas),
        "sum_delta_timesteps": sum(deltas),
        "mean_delta_timesteps": statistics.fmean(deltas) if deltas else None,
        "median_delta_timesteps": statistics.median(deltas) if deltas else None,
        "mean_relative_delta_fraction": (
            statistics.fmean(relative_deltas) if relative_deltas else None
        ),
        "median_relative_delta_fraction": (
            statistics.median(relative_deltas) if relative_deltas else None
        ),
    }


def random_effect_strata_rows(
    blocks: Mapping[tuple[int, int], Mapping[str, Trial]],
) -> list[dict[str, object]]:
    observations = random_effect_observations(blocks)
    families = (
        ("unstratified", ("all",)),
        ("realized_restart_count", ("none", "one_or_more")),
        (
            "accepted_restart_improvement_count",
            ("zero", "one_or_more"),
        ),
        (
            "joint_restart_outcome",
            (
                "no_fire",
                "fired_no_accepted_improvement",
                "fired_with_accepted_improvement",
            ),
        ),
    )
    scopes: list[
        tuple[str, float, int | None, list[dict[str, object]]]
    ] = []
    probabilities = sorted(
        {
            float(observation["configured_random_restart_probability"])
            for observation in observations
        }
    )
    for probability in probabilities:
        probability_observations = [
            observation
            for observation in observations
            if math.isclose(
                float(observation["configured_random_restart_probability"]),
                probability,
                rel_tol=0.0,
                abs_tol=1e-12,
            )
        ]
        probability_percent = int(round(100.0 * probability))
        depths = sorted(
            {int(observation["depth"]) for observation in probability_observations}
        )
        if len(depths) > 1:
            scopes.append(
                (
                    f"random_{probability_percent}_all_depths",
                    probability,
                    None,
                    probability_observations,
                )
            )
        scopes.extend(
            (
                f"random_{probability_percent}_depth_{depth}",
                probability,
                depth,
                [
                    observation
                    for observation in probability_observations
                    if observation["depth"] == depth
                ],
            )
            for depth in depths
        )

    rows: list[dict[str, object]] = []
    for scope, probability, depth, scope_observations in scopes:
        for family, strata in families:
            for stratum in strata:
                rows.append(
                    summarize_random_stratum(
                        scope_observations,
                        scope=scope,
                        probability=probability,
                        depth=depth,
                        family=family,
                        stratum=stratum,
                    )
                )
    return rows


def trial_rows(trials: Sequence[Trial], root: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for trial in trials:
        row: dict[str, object] = {
            "task": trial.task,
            "seed": trial.seed,
            "policy": trial.policy,
            "selective_replanning": trial.selective_replanning,
            "history_depth": trial.history_depth,
            "configured_random_restart_probability": trial.random_restart_probability,
            "initial_makespan_timesteps": trial.initial_makespan,
            "final_makespan_timesteps": trial.final_makespan,
            "improvement_timesteps": trial.improvement_timesteps,
            "improvement_fraction": trial.improvement_fraction,
            "planning_time_seconds": trial.planning_time_seconds,
            "setting_sources": trial.setting_sources,
            "trial_json": str(trial.path.relative_to(root)),
        }
        for field in COUNTER_FIELDS:
            sourced = getattr(trial, field)
            row[field] = sourced.value
            row[f"{field}_source"] = sourced.source
        row["conflict_detection_seconds"] = trial.conflict_detection_seconds.value
        row["conflict_detection_seconds_source"] = (
            trial.conflict_detection_seconds.source
        )
        rows.append(row)
    return rows


def paired_endpoint_rows(
    blocks: Mapping[tuple[int, int], Mapping[str, Trial]],
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for (task, seed), block in blocks.items():
        best = min(trial.final_makespan for trial in block.values())
        initial = next(iter(block.values())).initial_makespan
        full = block["full_restart"].final_makespan
        row: dict[str, object] = {
            "task": task,
            "seed": seed,
            "initial_makespan_timesteps": initial,
            "best_final_makespan_timesteps": best,
        }
        for policy in POLICIES:
            final = block[policy].final_makespan
            row[f"{policy}_final_makespan_timesteps"] = final
            row[f"{policy}_ratio_to_matched_best"] = final / best
            row[f"{policy}_delta_vs_full_restart_timesteps"] = final - full
        rows.append(row)
    return rows


def write_csv(
    path: Path,
    columns: Sequence[str],
    rows: Iterable[Mapping[str, object]],
) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns, extrasaction="raise")
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, value: object) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(value, handle, indent=2, sort_keys=True, allow_nan=False)
        handle.write("\n")


def markdown_number(value: object, digits: int = 2) -> str:
    if value is None:
        return "—"
    numeric = float(value)
    if numeric.is_integer():
        return str(int(numeric))
    return f"{numeric:.{digits}f}"


def markdown_percent(value: object, digits: int = 2) -> str:
    if value is None:
        return "—"
    return f"{100.0 * float(value):.{digits}f}%"


def comparison_label(row: Mapping[str, object]) -> str:
    return (
        f"{POLICY_LABELS[str(row['candidate'])]} vs "
        f"{POLICY_LABELS[str(row['reference'])]}"
    )


def markdown_comparison_table(rows: Sequence[Mapping[str, object]]) -> list[str]:
    lines = [
        "| Comparison | Wins / ties / losses | Mean delta | Median delta | Mean relative delta |",
        "|---|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            "| "
            + comparison_label(row)
            + " | "
            + f"{row['wins']} / {row['ties']} / {row['losses']}"
            + " | "
            + markdown_number(row["mean_delta_timesteps"])
            + " | "
            + markdown_number(row["median_delta_timesteps"])
            + " | "
            + markdown_percent(row["mean_relative_delta_fraction"])
            + " |"
        )
    return lines


RANDOM_STRATUM_LABELS = {
    "all": "All paired candidates",
    "none": "No random restart realized",
    "one_or_more": "One or more",
    "zero": "Zero",
    "no_fire": "No random restart realized",
    "fired_no_accepted_improvement": "Restart(s), no accepted improvement",
    "fired_with_accepted_improvement": "Restart(s), accepted improvement(s)",
}


def markdown_random_strata_table(
    rows: Sequence[Mapping[str, object]], family: str
) -> list[str]:
    selected_rows = [row for row in rows if row["stratum_family"] == family]
    lines = [
        "| Scope | Stratum | Pairs / telemetry eligible | Wins / ties / losses | Mean delta | Median delta |",
        "|---|---|---:|---:|---:|---:|",
    ]
    for row in selected_rows:
        probability = int(
            round(100.0 * float(row["configured_random_restart_probability"]))
        )
        scope = (
            f"{probability}% / all matching depths"
            if row["depth"] == "all"
            else f"{probability}% / depth {row['depth']}"
        )
        lines.append(
            f"| {scope} | {RANDOM_STRATUM_LABELS[str(row['stratum'])]} | "
            f"{row['pair_count']} / {row['telemetry_eligible_pair_count']} | "
            f"{row['wins']} / {row['ties']} / {row['losses']} | "
            f"{markdown_number(row['mean_delta_timesteps'])} | "
            f"{markdown_number(row['median_delta_timesteps'])} |"
        )
    return lines


def build_markdown(
    summaries: Mapping[str, Mapping[str, object]],
    comparisons: Sequence[Mapping[str, object]],
    random_strata: Sequence[Mapping[str, object]],
    artifacts: Mapping[str, str],
) -> str:
    overall_comparisons = [
        row for row in comparisons if row["scope"] == "all_tasks"
    ]
    vs_full = [
        row for row in overall_comparisons if row["category"] == "versus_full_restart"
    ]
    focal = [
        row for row in overall_comparisons if row["category"] != "versus_full_restart"
    ]

    lines = [
        "# AO-ARC expanded restart-policy analysis",
        "",
        "## Dataset validation",
        "",
        (
            "Validated exactly 400 completed, externally non-timeout, successful exact "
            "AO-ARC trials: five Panda Cage n=4 tasks (0–4), ten matched planning "
            "seeds (0–9), and eight restart policies. Every matched task/seed block "
            "has the same initial makespan across policies."
        ),
        "",
        "All endpoint comparisons are paired by `(task, seed)`. A delta is candidate "
        "minus reference, so a negative delta favors the candidate. Wins, ties, and "
        "losses use exact integer makespans.",
        "",
        "## Endpoint quality",
        "",
        "| Policy | Mean final | Median final | Mean improvement | Best or tied | Median / matched best |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for policy in POLICIES:
        summary = summaries[policy]
        lines.append(
            f"| {POLICY_LABELS[policy]} | "
            f"{markdown_number(summary['mean_final_makespan_timesteps'])} | "
            f"{markdown_number(summary['median_final_makespan_timesteps'])} | "
            f"{markdown_percent(summary['mean_initial_to_final_improvement_fraction'])} | "
            f"{summary['best_or_tied_count']}/50 | "
            f"{markdown_number(summary['median_ratio_to_matched_best'], 4)}× |"
        )

    lines.extend(
        [
            "",
            "## Paired comparisons against full restart",
            "",
            *markdown_comparison_table(vs_full),
            "",
            "## Depth and random-restart contrasts",
            "",
            *markdown_comparison_table(focal),
            "",
            "## Paired random-effect strata",
            "",
            (
                "These rows compare each stochastic random-restart trial with its matched "
                "zero-random trial at the same history depth, task, and seed. Negative "
                "deltas favor random restart. Probability is always part of the scope: "
                "25% and 50% observations are never pooled. The 25% all-depth scope "
                "contains depth 0, 1, and 2 contrasts; every depth-specific row contains "
                "50 matched contrasts."
            ),
            "",
            "### Unstratified random effect",
            "",
            *markdown_random_strata_table(random_strata, "unstratified"),
            "",
            "### Grouped by realized random restarts",
            "",
            *markdown_random_strata_table(
                random_strata, "realized_restart_count"
            ),
            "",
            "### Grouped by accepted random-restart improvements",
            "",
            (
                "The zero-improvement stratum includes trials where no restart fired. "
                "The joint table below separates those cases."
            ),
            "",
            *markdown_random_strata_table(
                random_strata, "accepted_restart_improvement_count"
            ),
            "",
            "### Mutually exclusive restart outcomes",
            "",
            *markdown_random_strata_table(random_strata, "joint_restart_outcome"),
            "",
            (
                "The no-fire rows expose identity and fixed-budget timing variation. "
                "Fired-without-accepted-improvement rows isolate runs that paid restart "
                "budget without recording a direct accepted gain. Fired-with-accepted-"
                "improvement rows identify runs in which the escape mechanism produced "
                "at least one accepted strict makespan improvement. These remain "
                "descriptive post-treatment strata, not causal subgroups."
            ),
            "",
            "## Per-task paired behavior",
            "",
            "The following rows expose whether aggregate results are broad or driven by "
            "specific task families. Each row contains ten matched seeds.",
            "",
            "| Task | Comparison | Wins / ties / losses | Mean delta | Median delta |",
            "|---:|---|---:|---:|---:|",
        ]
    )
    for task in EXPECTED_TASKS:
        task_rows = [
            row for row in comparisons if row["scope"] == f"task_{task}"
        ]
        for row in task_rows:
            lines.append(
                f"| {task} | {comparison_label(row)} | "
                f"{row['wins']} / {row['ties']} / {row['losses']} | "
                f"{markdown_number(row['mean_delta_timesteps'])} | "
                f"{markdown_number(row['median_delta_timesteps'])} |"
            )

    lines.extend(
        [
            "",
            "## AO work and restart realization",
            "",
            (
                "`Random count / calls` counts stochastic full-restart decisions among "
                "bounded ARC calls. `Accepted / random` is the fraction of realized "
                "stochastic restarts that produced an accepted strict makespan "
                "improvement. Neither measure relabels ordinary calls made by the "
                "full-restart baseline as random restarts."
            ),
            "",
            "| Policy | Attempts | Improvements | Random count / calls (rate) | Accepted / random (rate) | Paths replanned | Paths reused | Conflict pairs skipped | History-expanded robots |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for policy in POLICIES:
        summary = summaries[policy]
        random_total = summary["random_full_restarts_total"]
        random_calls = summary["random_restart_decision_calls"]
        random_cell = "—"
        if random_total is not None and random_calls:
            random_cell = (
                f"{random_total} / {random_calls} "
                f"({markdown_percent(summary['realized_random_restart_rate'])})"
            )
        random_improvement_total = summary["random_restart_improvements_for_rate"]
        random_improvement_denominator = summary[
            "random_restart_improvement_denominator"
        ]
        random_improvement_cell = "—"
        if (
            random_improvement_total is not None
            and random_improvement_denominator
        ):
            random_improvement_cell = (
                f"{random_improvement_total} / {random_improvement_denominator} "
                f"({markdown_percent(summary['accepted_improvement_rate_per_random_restart'])})"
            )
        lines.append(
            f"| {POLICY_LABELS[policy]} | "
            f"{markdown_number(summary['attempts_total'])} | "
            f"{markdown_number(summary['improvements_total'])} | "
            f"{random_cell} | "
            f"{random_improvement_cell} | "
            f"{markdown_number(summary['paths_replanned_total'])} | "
            f"{markdown_number(summary['paths_reused_total'])} | "
            f"{markdown_number(summary['conflict_pairs_skipped_total'])} | "
            f"{markdown_number(summary['history_expanded_robots_total'])} |"
        )

    lines.extend(
        [
            "",
            "## Telemetry completeness",
            "",
            (
                "Counts below are trials with a usable value out of 50. `Recorded` also "
                "includes values cross-checked against per-attempt data; `derived` means "
                "only per-attempt data supplied the total. Legacy zero-probability runs "
                "may have restart and restart-improvement counts inferred as zero because "
                "their configuration proves that no stochastic restart occurred."
            ),
            "",
            "| Policy | Attempts | Improvements | Random restarts | Random-restart improvements | Replanned | Reused | Pairs skipped |",
            "|---|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for policy in POLICIES:
        availability_data = summaries[policy]["telemetry_availability"]

        def coverage(field: str) -> str:
            field_data = availability_data[field]
            suffix = ""
            if field_data["inferred"]:
                suffix = f" ({field_data['inferred']} inferred)"
            return f"{field_data['available']}/50{suffix}"

        lines.append(
            f"| {POLICY_LABELS[policy]} | {coverage('attempts')} | "
            f"{coverage('improvements')} | {coverage('random_full_restarts')} | "
            f"{coverage('random_full_restart_improvements')} | "
            f"{coverage('paths_replanned')} | {coverage('paths_reused')} | "
            f"{coverage('conflict_pairs_skipped')} |"
        )

    lines.extend(
        [
            "",
            "## Interpretation limits",
            "",
            "- The tables test the observed local-minimum/escape behavior descriptively; "
            "they do not by themselves establish why a particular incumbent was found.",
            "- `num_paths_replanned` counts initial bounded-path replans completed by an "
            "attempt. It excludes an in-progress path interrupted by the budget and paths "
            "changed later during conflict repair.",
            "- Random restart realization is expected to vary around its configured 25% or "
            "50% probability; paired quality comparisons evaluate the configured policy, "
            "not a forced exact event count. The accepted-improvement rate is undefined "
            "when no stochastic restart was realized.",
            "- Missing legacy fields remain unavailable unless they can be derived exactly. "
            "The detailed JSON and trial CSV retain source labels for every counter.",
            "",
            "## Artifacts",
            "",
            f"- Trial telemetry: [{artifacts['trials_csv']}]({artifacts['trials_csv']})",
            f"- Paired endpoints: [{artifacts['paired_csv']}]({artifacts['paired_csv']})",
            f"- Policy summaries: [{artifacts['policy_csv']}]({artifacts['policy_csv']})",
            f"- Pairwise summaries: [{artifacts['comparisons_csv']}]({artifacts['comparisons_csv']})",
            f"- Random-effect strata: [{artifacts['random_strata_csv']}]({artifacts['random_strata_csv']})",
            f"- Machine-readable analysis: [{artifacts['json']}]({artifacts['json']})",
            "",
        ]
    )
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    root = args.root.resolve()
    output_dir = (
        args.output_dir.resolve()
        if args.output_dir is not None
        else root / "expanded_restart_policy_analysis"
    )

    try:
        trials = load_trials(root)
        blocks = group_blocks(trials)
        summaries, summaries_by_task = policy_summary_by_scope(trials, blocks)
        comparisons = comparison_rows(blocks)
        random_strata = random_effect_strata_rows(blocks)
    except AnalysisError as exc:
        print(f"analysis error: {exc}", file=sys.stderr)
        return 2

    output_dir.mkdir(parents=True, exist_ok=True)
    paths = {
        "markdown": output_dir / f"{OUTPUT_STEM}_analysis.md",
        "trials_csv": output_dir / f"{OUTPUT_STEM}_trials.csv",
        "paired_csv": output_dir / f"{OUTPUT_STEM}_paired_endpoints.csv",
        "policy_csv": output_dir / f"{OUTPUT_STEM}_policy_summary.csv",
        "comparisons_csv": output_dir / f"{OUTPUT_STEM}_pairwise_summary.csv",
        "random_strata_csv": output_dir
        / f"{OUTPUT_STEM}_random_effect_strata.csv",
        "json": output_dir / f"{OUTPUT_STEM}_analysis.json",
    }

    trial_data = trial_rows(trials, root)
    trial_columns = [
        "task",
        "seed",
        "policy",
        "selective_replanning",
        "history_depth",
        "configured_random_restart_probability",
        "initial_makespan_timesteps",
        "final_makespan_timesteps",
        "improvement_timesteps",
        "improvement_fraction",
        "planning_time_seconds",
        "setting_sources",
    ]
    for field in COUNTER_FIELDS:
        trial_columns.extend((field, f"{field}_source"))
    trial_columns.extend(
        (
            "conflict_detection_seconds",
            "conflict_detection_seconds_source",
            "trial_json",
        )
    )
    write_csv(paths["trials_csv"], trial_columns, trial_data)

    paired_data = paired_endpoint_rows(blocks)
    paired_columns = [
        "task",
        "seed",
        "initial_makespan_timesteps",
        "best_final_makespan_timesteps",
    ]
    for policy in POLICIES:
        paired_columns.extend(
            (
                f"{policy}_final_makespan_timesteps",
                f"{policy}_ratio_to_matched_best",
                f"{policy}_delta_vs_full_restart_timesteps",
            )
        )
    write_csv(paths["paired_csv"], paired_columns, paired_data)

    policy_columns = [
        "policy",
        "label",
        "selective_replanning",
        "history_depth",
        "configured_random_restart_probability",
        "trial_count",
        "mean_initial_makespan_timesteps",
        "median_initial_makespan_timesteps",
        "mean_final_makespan_timesteps",
        "median_final_makespan_timesteps",
        "mean_initial_to_final_improvement_fraction",
        "median_final_over_initial",
        "mean_ratio_to_matched_best",
        "median_ratio_to_matched_best",
        "best_or_tied_count",
    ]
    for field in COUNTER_FIELDS:
        policy_columns.extend((f"{field}_total", f"{field}_trials_available"))
    policy_columns.extend(
        (
            "improvements_per_bounded_attempt",
            "random_restart_decision_calls",
            "realized_random_restart_rate",
            "random_restart_improvements_for_rate",
            "random_restart_improvement_denominator",
            "accepted_improvement_rate_per_random_restart",
            "conflict_detection_seconds_total",
            "conflict_detection_seconds_trials_available",
        )
    )
    policy_rows = [
        {column: summaries[policy].get(column) for column in policy_columns}
        for policy in POLICIES
    ]
    write_csv(paths["policy_csv"], policy_columns, policy_rows)

    comparison_columns = [
        "scope",
        "category",
        "comparison",
        "candidate",
        "reference",
        "pair_count",
        "wins",
        "ties",
        "losses",
        "sum_delta_timesteps",
        "mean_delta_timesteps",
        "median_delta_timesteps",
        "mean_relative_delta_fraction",
        "median_relative_delta_fraction",
        "minimum_delta_timesteps",
        "maximum_delta_timesteps",
    ]
    write_csv(paths["comparisons_csv"], comparison_columns, comparisons)

    random_strata_columns = [
        "scope",
        "configured_random_restart_probability",
        "depth",
        "candidate",
        "reference",
        "stratum_family",
        "stratum",
        "scope_pair_count",
        "telemetry_eligible_pair_count",
        "telemetry_unavailable_pair_count",
        "pair_count",
        "wins",
        "ties",
        "losses",
        "sum_delta_timesteps",
        "mean_delta_timesteps",
        "median_delta_timesteps",
        "mean_relative_delta_fraction",
        "median_relative_delta_fraction",
    ]
    write_csv(
        paths["random_strata_csv"],
        random_strata_columns,
        random_strata,
    )

    artifacts = {name: path.name for name, path in paths.items()}
    analysis_json = {
        "schema": "comotion.ao_arc_expanded_restart_policy_analysis.v2",
        "design": {
            "scenario": "panda_cage_n4",
            "tasks": list(EXPECTED_TASKS),
            "seeds": list(EXPECTED_SEEDS),
            "policies": list(POLICIES),
            "time_limit_seconds": EXPECTED_TIME_LIMIT_SECONDS,
            "trial_count": len(trials),
            "paired_block_count": len(blocks),
            "comparison_delta_definition": "candidate final makespan minus reference final makespan",
        },
        "policy_summary": summaries,
        "policy_summary_by_task": summaries_by_task,
        "pairwise_comparisons": comparisons,
        "random_effect_strata": random_strata,
        "artifacts": artifacts,
    }
    write_json(paths["json"], analysis_json)
    markdown = build_markdown(summaries, comparisons, random_strata, artifacts)
    with paths["markdown"].open("w", encoding="utf-8") as handle:
        handle.write(markdown)

    print(f"validated_trials: {len(trials)}")
    print(f"paired_blocks: {len(blocks)}")
    for name, path in paths.items():
        print(f"{name}: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
