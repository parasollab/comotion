#!/usr/bin/env python3
"""Plot paired endpoint and anytime comparisons for AO-ARC restart policies."""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import os
import re
import statistics
import tempfile
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
    "violators_only": "Violators only",
    "violators_only_random_25": "Violators only + 25% random restart",
    "violators_only_random_50": "Violators only + 50% random restart",
    "history_one_hop": "One-hop history",
    "history_one_hop_random_25": "One-hop + 25% random restart",
    "history_two_hop": "Two-hop history",
    "history_two_hop_random_25": "Two-hop + 25% random restart",
}
POLICY_SHORT_LABELS = {
    "full_restart": "Full",
    "violators_only": "Viol.",
    "violators_only_random_25": "Viol R25",
    "violators_only_random_50": "Viol R50",
    "history_one_hop": "1-hop",
    "history_one_hop_random_25": "1-hop R25",
    "history_two_hop": "2-hop",
    "history_two_hop_random_25": "2-hop R25",
}
POLICY_COLORS = {
    "full_restart": "#0072B2",
    "violators_only": "#D55E00",
    "violators_only_random_25": "#17A8B8",
    "violators_only_random_50": "#8C564B",
    "history_one_hop": "#009E73",
    "history_one_hop_random_25": "#E69F00",
    "history_two_hop": "#CC79A7",
    "history_two_hop_random_25": "#6F42C1",
}
POLICY_LINESTYLES = {
    "full_restart": "-",
    "violators_only": "--",
    "violators_only_random_25": (0, (7, 1.4, 1.2, 1.4)),
    "violators_only_random_50": (0, (2, 1.1)),
    "history_one_hop": "-.",
    "history_one_hop_random_25": (0, (5, 1.5)),
    "history_two_hop": ":",
    "history_two_hop_random_25": (0, (3, 1.2, 1, 1.2)),
}
POLICY_SETTINGS = {
    "full_restart": (False, 0, 0.0),
    "violators_only": (True, 0, 0.0),
    "violators_only_random_25": (True, 0, 0.25),
    "violators_only_random_50": (True, 0, 0.50),
    "history_one_hop": (True, 1, 0.0),
    "history_one_hop_random_25": (True, 1, 0.25),
    "history_two_hop": (True, 2, 0.0),
    "history_two_hop_random_25": (True, 2, 0.25),
}
# These three policies predate the numeric depth/probability options. Their
# archived records may omit those fields, so absent values are inferred from
# the legacy Boolean settings. Every newer policy must record each setting.
LEGACY_INFERRED_POLICIES = {
    "full_restart",
    "violators_only",
    "history_one_hop",
}
EXPECTED_TASKS = tuple(range(5))
EXPECTED_SEEDS = tuple(range(10))
EXPECTED_TIME_LIMIT_SECONDS = 90.0
RATIO_THRESHOLDS = (1.0, 1.05, 1.10, 1.25, 1.50, 2.0)


@dataclass(frozen=True)
class TrialTrace:
    task: int
    seed: int
    policy: str
    initial_makespan: int
    final_makespan: int
    first_solution_time: float
    planning_time: float
    time_limit: float
    elapsed_since_initial: tuple[float, ...]
    makespans: tuple[int, ...]

    @property
    def optimization_runtime(self) -> float:
        return self.planning_time - self.first_solution_time

    def makespan_at(self, elapsed: float) -> int:
        index = bisect.bisect_right(self.elapsed_since_initial, elapsed) - 1
        if index < 0:
            raise RuntimeError("Aligned trace has no incumbent at nonnegative time")
        return self.makespans[index]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a paired final-makespan ECDF and aligned median-makespan "
            "anytime plot from an AO-ARC restart-policy parameter sweep."
        )
    )
    parser.add_argument(
        "result_root",
        type=Path,
        help="Sweep result root containing trials/**/trial.json.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Output directory (defaults to RESULT_ROOT/plots).",
    )
    return parser.parse_args()


def finite_float(value: object, field: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"Invalid {field}: {value!r}") from exc
    if not math.isfinite(result):
        raise RuntimeError(f"Non-finite {field}: {value!r}")
    return result


def object_value(value: object, field: str, path: Path) -> dict[str, object]:
    if not isinstance(value, dict):
        raise RuntimeError(f"Expected object {field} in {path}")
    return value


def optional_object(value: object) -> dict[str, object]:
    return value if isinstance(value, dict) else {}


def nonnegative_int(value: object, field: str, path: Path) -> int:
    if isinstance(value, bool):
        raise RuntimeError(f"Invalid integer {field}={value!r} in {path}")
    try:
        result = int(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise RuntimeError(f"Invalid integer {field}={value!r} in {path}") from exc
    if result < 0 or isinstance(value, float) and not value.is_integer():
        raise RuntimeError(f"Invalid nonnegative integer {field}={value!r} in {path}")
    return result


def positive_int(value: object, field: str, path: Path) -> int:
    result = nonnegative_int(value, field, path)
    if result == 0:
        raise RuntimeError(f"Expected positive integer {field} in {path}")
    return result


def explicit_bool(value: object, field: str, path: Path) -> bool:
    if not isinstance(value, bool):
        raise RuntimeError(f"Expected Boolean {field} in {path}, found {value!r}")
    return value


def command_option(command: object, option: str, path: Path) -> str | None:
    if not isinstance(command, list):
        return None
    words = [str(word) for word in command]
    indexes = [index for index, word in enumerate(words) if word == option]
    if not indexes:
        return None
    if len(indexes) != 1:
        raise RuntimeError(f"Repeated command option {option} in {path}")
    index = indexes[0]
    if index + 1 >= len(words):
        raise RuntimeError(f"Missing value after command option {option} in {path}")
    return words[index + 1]


def command_switch(
    command: object,
    positive: str,
    negative: str,
    path: Path,
) -> bool | None:
    if not isinstance(command, list):
        return None
    words = {str(word) for word in command}
    has_positive = positive in words
    has_negative = negative in words
    if has_positive and has_negative:
        raise RuntimeError(
            f"Conflicting command switches {positive} and {negative} in {path}"
        )
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
) -> object:
    present = [(source, value) for source, value in values if value is not None]
    if not present:
        return expected
    first_value = present[0][1]
    if any(value != first_value for _source, value in present[1:]):
        raise RuntimeError(
            f"Conflicting {field} settings in {path}: "
            + ", ".join(f"{source}={value!r}" for source, value in present)
        )
    if first_value != expected:
        raise RuntimeError(
            f"Policy setting mismatch in {path}: {field}={first_value!r}, "
            f"expected {expected!r}"
        )
    return first_value


def validate_policy_settings(
    trial: Mapping[str, object],
    stats: Mapping[str, object],
    params: Mapping[str, object],
    policy: str,
    path: Path,
) -> None:
    """Validate policy settings, accepting the same legacy records as the analyzer."""
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
        path,
    )
    reconcile_setting(
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
        raise RuntimeError(
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
        command, "--ao-arc-repair-history-replanning-depth", path
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

    # Legacy trials represented exactly depth one with a Boolean flag.
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
        path,
    )
    if command_expansion is not None:
        legacy_expansion_values.append(("command_legacy_bool", command_expansion))
    if depth_values:
        explicit_depth = depth_values[0][1]
        for source, enabled in legacy_expansion_values:
            if enabled != (explicit_depth != 0):
                raise RuntimeError(
                    f"Conflicting history depth and {source} in {path}"
                )
    else:
        depth_values.extend(
            (source, 1 if enabled else 0)
            for source, enabled in legacy_expansion_values
        )
    if policy not in LEGACY_INFERRED_POLICIES and not depth_values:
        raise RuntimeError(
            f"Missing explicit repair-history replanning depth in {path}"
        )
    reconcile_setting(
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
                ),
            )
        )
    command_probability = command_option(
        command, "--ao-arc-random-full-restart-probability", path
    )
    if command_probability is not None:
        probability_values.append(
            (
                "command",
                finite_float(
                    command_probability,
                    "--ao-arc-random-full-restart-probability",
                ),
            )
        )
    if policy not in LEGACY_INFERRED_POLICIES and not probability_values:
        raise RuntimeError(
            f"Missing explicit random full-restart probability in {path}"
        )
    if any(
        not math.isclose(
            float(value), expected_probability, rel_tol=0.0, abs_tol=1e-12
        )
        for _source, value in probability_values
    ):
        raise RuntimeError(
            f"Policy random-restart probability mismatch in {path}: "
            + ", ".join(
                f"{source}={value!r}" for source, value in probability_values
            )
            + f", expected {expected_probability}"
        )


def load_trial(path: Path) -> TrialTrace:
    try:
        with path.open(encoding="utf-8") as handle:
            trial = json.load(handle)
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"Unable to read {path}: {exc}") from exc

    trial = object_value(trial, "trial", path)
    row = object_value(trial.get("result_row"), "result_row", path)
    metrics = object_value(trial.get("metrics"), "metrics", path)
    stats = optional_object(metrics.get("planner_stats"))
    params = optional_object(trial.get("params"))
    context = optional_object(metrics.get("benchmark_context"))
    policy = str(row.get("param_set", ""))
    if policy not in POLICIES:
        raise RuntimeError(f"Unexpected policy in {path}: {policy!r}")
    if trial.get("status") != "complete":
        raise RuntimeError(f"Trial is not complete: {path}")
    if trial.get("returncode") != 0 or row.get("returncode") != 0:
        raise RuntimeError(f"Trial did not return zero: {path}")
    if trial.get("timed_out") is not False or row.get("timed_out") is not False:
        raise RuntimeError(f"Trial hit its external timeout: {path}")
    if row.get("success") is not True or metrics.get("success") is not True:
        raise RuntimeError(f"Trial has no successful incumbent: {path}")
    if str(row.get("method", "")) != "ao_arc":
        raise RuntimeError(f"Expected AO-ARC method in {path}")
    if "exact" not in str(metrics.get("planner_status", "")).lower():
        raise RuntimeError(f"Expected exact planner status in {path}")

    app = str(row.get("app", ""))
    app_match = re.fullmatch(r"panda_cage_n4_task([0-4])", app)
    if not app_match:
        raise RuntimeError(f"Unexpected Panda Cage app {app!r} in {path}")
    app_task = int(app_match.group(1))
    context_task = context.get("task_index")
    task = (
        app_task
        if context_task is None
        else nonnegative_int(context_task, "task_index", path)
    )
    if task != app_task:
        raise RuntimeError(f"App/context task mismatch in {path}")
    seed = nonnegative_int(row.get("seed"), "seed", path)
    if "num_robots" not in context:
        raise RuntimeError(f"Missing benchmark_context.num_robots in {path}")
    if nonnegative_int(context["num_robots"], "num_robots", path) != 4:
        raise RuntimeError(f"Expected four robots in {path}")
    if "time_limit_seconds" not in context:
        raise RuntimeError(
            f"Missing benchmark_context.time_limit_seconds in {path}"
        )
    time_limit = finite_float(
        context["time_limit_seconds"], "time_limit_seconds"
    )
    if not math.isclose(
        time_limit,
        EXPECTED_TIME_LIMIT_SECONDS,
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise RuntimeError(f"Expected a 90-second planning limit in {path}")

    validate_policy_settings(trial, stats, params, policy, path)
    if "selective_initial_conflict_scan" in stats and not explicit_bool(
        stats["selective_initial_conflict_scan"],
        "selective_initial_conflict_scan",
        path,
    ):
        raise RuntimeError(f"Selective initial conflict scan is disabled in {path}")

    raw_events = stats.get("solution_events")
    if not isinstance(raw_events, list) or not raw_events:
        raise RuntimeError(f"Trial has no solution events: {path}")

    event_times: list[float] = []
    makespans: list[int] = []
    for index, event_value in enumerate(raw_events):
        event = object_value(event_value, f"solution_events[{index}]", path)
        event_times.append(
            finite_float(
                event.get("elapsed_seconds"),
                f"solution_events[{index}].elapsed_seconds",
            )
        )
        makespans.append(
            positive_int(
                event.get("makespan_timesteps"),
                f"solution_events[{index}].makespan_timesteps",
                path,
            )
        )
    if event_times[0] < 0.0 or any(
        later <= earlier for earlier, later in zip(event_times, event_times[1:])
    ):
        raise RuntimeError(f"Solution-event times are not strictly increasing: {path}")
    if any(later >= earlier for earlier, later in zip(makespans, makespans[1:])):
        raise RuntimeError(f"Solution-event makespans are not strictly decreasing: {path}")
    metrics_final = positive_int(
        metrics.get("makespan_timesteps"), "makespan_timesteps", path
    )
    if makespans[-1] != metrics_final:
        raise RuntimeError(f"Final solution event does not match final makespan: {path}")

    first_time = event_times[0]
    aligned_times = tuple(value - first_time for value in event_times)
    planning_time = finite_float(metrics.get("planning_time_seconds"), "planning time")
    if planning_time < event_times[-1]:
        raise RuntimeError(f"Final event occurs after recorded planning time: {path}")

    return TrialTrace(
        task=task,
        seed=seed,
        policy=policy,
        initial_makespan=makespans[0],
        final_makespan=makespans[-1],
        first_solution_time=first_time,
        planning_time=planning_time,
        time_limit=time_limit,
        elapsed_since_initial=aligned_times,
        makespans=tuple(makespans),
    )


def load_traces(result_root: Path) -> list[TrialTrace]:
    paths = sorted((result_root / "trials").rglob("trial.json"))
    if not paths:
        raise RuntimeError(f"No trial.json files found below {result_root / 'trials'}")
    traces = [load_trial(path) for path in paths]

    seen: set[tuple[int, int, str]] = set()
    blocks: dict[tuple[int, int], dict[str, TrialTrace]] = {}
    for trace in traces:
        identity = (trace.task, trace.seed, trace.policy)
        if identity in seen:
            raise RuntimeError(f"Duplicate trial identity: {identity}")
        seen.add(identity)
        blocks.setdefault((trace.task, trace.seed), {})[trace.policy] = trace

    expected = set(POLICIES)
    for block, policy_traces in blocks.items():
        if set(policy_traces) != expected:
            raise RuntimeError(
                f"Matched block {block} has policies {sorted(policy_traces)}, "
                f"expected {sorted(expected)}"
            )
        initial_values = {trace.initial_makespan for trace in policy_traces.values()}
        if len(initial_values) != 1:
            raise RuntimeError(f"Policies have different initial makespans in {block}")

    expected_blocks = {
        (task, seed)
        for task in EXPECTED_TASKS
        for seed in EXPECTED_SEEDS
    }
    actual_blocks = set(blocks)
    expected_trial_count = len(expected_blocks) * len(POLICIES)
    if actual_blocks != expected_blocks or len(traces) != expected_trial_count:
        missing = sorted(expected_blocks - actual_blocks)
        extra = sorted(actual_blocks - expected_blocks)
        raise RuntimeError(
            "Expected exactly tasks 0–4 × seeds 0–9 × eight policies; "
            f"found {len(traces)} trials ({len(blocks)} blocks), "
            f"missing blocks={missing}, extra blocks={extra}"
        )

    time_limits = {trace.time_limit for trace in traces}
    if len(time_limits) != 1:
        raise RuntimeError(f"Trials have inconsistent time limits: {sorted(time_limits)}")
    if not math.isclose(
        next(iter(time_limits)),
        EXPECTED_TIME_LIMIT_SECONDS,
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise RuntimeError(f"Expected a 90-second time limit, found {sorted(time_limits)}")
    return traces


def group_by_block(
    traces: Sequence[TrialTrace],
) -> dict[tuple[int, int], dict[str, TrialTrace]]:
    blocks: dict[tuple[int, int], dict[str, TrialTrace]] = {}
    for trace in traces:
        blocks.setdefault((trace.task, trace.seed), {})[trace.policy] = trace
    return dict(sorted(blocks.items()))


def write_csv(path: Path, columns: Sequence[str], rows: Iterable[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def paired_ratio_rows(
    blocks: dict[tuple[int, int], dict[str, TrialTrace]],
) -> tuple[list[dict[str, object]], dict[str, list[float]]]:
    rows: list[dict[str, object]] = []
    ratios = {policy: [] for policy in POLICIES}
    for (task, seed), policy_traces in blocks.items():
        best = min(trace.final_makespan for trace in policy_traces.values())
        initial = next(iter(policy_traces.values())).initial_makespan
        row: dict[str, object] = {
            "task": task,
            "seed": seed,
            "initial_makespan_timesteps": initial,
            "best_final_makespan_timesteps": best,
        }
        for policy in POLICIES:
            final = policy_traces[policy].final_makespan
            ratio = final / best
            ratios[policy].append(ratio)
            row[f"{policy}_final_makespan_timesteps"] = final
            row[f"{policy}_ratio_to_matched_best"] = f"{ratio:.12g}"
        rows.append(row)
    return rows, ratios


def trace_grid(traces: Sequence[TrialTrace], horizon: float) -> list[float]:
    grid = {0.0, horizon}
    for checkpoint in (0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 30.0, 60.0):
        if checkpoint <= horizon:
            grid.add(checkpoint)
    for trace in traces:
        grid.update(
            elapsed
            for elapsed in trace.elapsed_since_initial
            if 0.0 <= elapsed <= horizon
        )
    return sorted(grid)


def median_trace(
    traces: Sequence[TrialTrace],
    grid: Sequence[float],
    *,
    normalize: bool,
) -> list[float]:
    result = []
    for elapsed in grid:
        values = [
            trace.makespan_at(elapsed) / trace.initial_makespan
            if normalize
            else float(trace.makespan_at(elapsed))
            for trace in traces
        ]
        result.append(float(statistics.median(values)))
    return result


def runtime_rows(
    traces: Sequence[TrialTrace],
    horizon: float,
) -> tuple[list[dict[str, object]], dict[str, dict[str, tuple[list[float], list[float]]]]]:
    tasks = sorted({trace.task for trace in traces})
    scopes: list[tuple[str, list[TrialTrace]]] = [
        (f"task_{task}", [trace for trace in traces if trace.task == task])
        for task in tasks
    ]
    scopes.append(("all_tasks", list(traces)))
    output_rows: list[dict[str, object]] = []
    plot_data: dict[str, dict[str, tuple[list[float], list[float]]]] = {}
    for scope_name, scope_traces in scopes:
        plot_data[scope_name] = {}
        for policy in POLICIES:
            policy_traces = [trace for trace in scope_traces if trace.policy == policy]
            grid = trace_grid(policy_traces, horizon)
            raw = median_trace(policy_traces, grid, normalize=False)
            normalized = median_trace(policy_traces, grid, normalize=True)
            plot_data[scope_name][policy] = (grid, raw)
            for elapsed, raw_value, normalized_value in zip(grid, raw, normalized):
                output_rows.append(
                    {
                        "scope": scope_name,
                        "elapsed_since_initial_seconds": f"{elapsed:.12g}",
                        "policy": policy,
                        "median_makespan_timesteps": f"{raw_value:.12g}",
                        "median_fraction_of_initial": f"{normalized_value:.12g}",
                        "num_traces": len(policy_traces),
                    }
                )
    return output_rows, plot_data


def ecdf_trace(values: Sequence[float], left: float, right: float) -> tuple[list[float], list[float]]:
    ordered = sorted(values)
    count = len(ordered)
    x_values = [left, *ordered, right]
    y_values = [0.0, *(100.0 * index / count for index in range(1, count + 1)), 100.0]
    return x_values, y_values


def ratio_axis_spec(ratios: dict[str, list[float]]) -> tuple[float, float, list[float]]:
    """Return log-axis bounds and readable ticks covering the observed ratios."""
    observed_max = max(max(values) for values in ratios.values())
    padded_max = max(1.10, observed_max * 1.025)
    candidates = [1.0, 1.10, 1.25, 1.50, 2.0, 3.0, 4.0, 5.0, 7.5, 10.0]
    decade = 10.0
    while candidates[-1] < padded_max:
        candidates.extend((2.0 * decade, 3.0 * decade, 5.0 * decade, 10.0 * decade))
        decade *= 10.0
    right = next(value for value in candidates if value >= padded_max)
    return 0.995, right, [value for value in candidates if value <= right]


def format_ratio_tick(value: float) -> str:
    if value < 2.0:
        return f"{value:.2f}×"
    if value.is_integer():
        return f"{value:.0f}×"
    return f"{value:g}×"


def endpoint_annotation(parts: Sequence[str]) -> str:
    rows = ["  ·  ".join(parts[index : index + 2]) for index in range(0, len(parts), 2)]
    return "At common horizon\n" + "\n".join(rows)


def configure_matplotlib(cache_dir: str):
    os.environ["MPLCONFIGDIR"] = cache_dir
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 10.5,
            "axes.edgecolor": "#334155",
            "axes.labelcolor": "#1E293B",
            "axes.titlecolor": "#0F172A",
            "xtick.color": "#334155",
            "ytick.color": "#334155",
            "legend.edgecolor": "#CBD5E1",
            "pdf.fonttype": 42,
            "svg.fonttype": "none",
        }
    )
    return plt


def style_axis(axis) -> None:
    axis.set_facecolor("#FFFFFF")
    axis.spines["top"].set_visible(False)
    axis.spines["right"].set_visible(False)
    axis.grid(True, which="major", color="#CBD5E1", alpha=0.72, linewidth=0.8)


def save_figure(figure, output_base: Path) -> list[Path]:
    paths = [output_base.with_suffix(extension) for extension in (".png", ".svg", ".pdf")]
    for path in paths:
        figure.savefig(path, bbox_inches="tight", facecolor=figure.get_facecolor())
    return paths


def plot_paired_cdf(plt, ratios: dict[str, list[float]], output_dir: Path) -> list[Path]:
    left, right, ticks = ratio_axis_spec(ratios)
    figure, axis = plt.subplots(figsize=(13.2, 7.6), dpi=180)
    figure.patch.set_facecolor("#F8FAFC")
    style_axis(axis)

    for policy in POLICIES:
        values = ratios[policy]
        block_count = len(values)
        x_values, y_values = ecdf_trace(values, left, right)
        best_count = sum(math.isclose(value, 1.0, abs_tol=1e-12) for value in values)
        axis.step(
            x_values,
            y_values,
            where="post",
            color=POLICY_COLORS[policy],
            linestyle=POLICY_LINESTYLES[policy],
            linewidth=2.7,
            label=(
                f"{POLICY_LABELS[policy]}  "
                f"({best_count}/{block_count} best or tied)"
            ),
        )

    axis.axhline(50.0, color="#94A3B8", linewidth=1.0, linestyle=(0, (3, 3)))
    axis.axvline(1.0, color="#64748B", linewidth=1.1, linestyle=(0, (3, 3)))
    axis.axvline(1.10, color="#94A3B8", linewidth=1.0, linestyle=":")
    axis.set_xscale("log")
    axis.set_xlim(left, right)
    axis.set_ylim(0.0, 102.0)
    axis.set_xticks(ticks)
    axis.set_xticklabels([format_ratio_tick(value) for value in ticks])
    axis.set_yticks([0, 20, 40, 60, 80, 100])
    axis.set_yticklabels(["0%", "20%", "40%", "60%", "80%", "100%"])
    axis.set_xlabel("Final makespan / best makespan in matched block  (lower is better)")
    axis.set_ylabel("Fraction of matched task–seed blocks")
    axis.legend(
        loc="lower right",
        ncol=2,
        fontsize=8.6,
        frameon=True,
        facecolor="#FFFFFF",
        framealpha=0.96,
    )

    summary_lines = ["Within 1.10× of best"]
    for policy in POLICIES:
        count = sum(value <= 1.10 + 1e-12 for value in ratios[policy])
        block_count = len(ratios[policy])
        percentage = round(100.0 * count / block_count)
        summary_lines.append(
            f"{POLICY_SHORT_LABELS[policy]:<10} {count:>2}/{block_count}  "
            f"({percentage:>2}%)"
        )
    axis.text(
        0.035,
        0.54,
        "\n".join(summary_lines),
        transform=axis.transAxes,
        ha="left",
        va="top",
        fontsize=8.8,
        family="DejaVu Sans Mono",
        color="#334155",
        bbox={"boxstyle": "round,pad=0.5", "facecolor": "#F8FAFC", "edgecolor": "#CBD5E1"},
    )

    figure.suptitle(
        "Paired CDF of final makespan",
        x=0.075,
        y=0.975,
        ha="left",
        fontsize=17,
        fontweight="bold",
        color="#0F172A",
    )
    figure.text(
        0.075,
        0.922,
        "Panda Cage n=4  •  5 tasks × 10 planning seeds  •  50 matched blocks  •  90 s",
        ha="left",
        fontsize=10.4,
        color="#475569",
    )
    figure.text(
        0.075,
        0.018,
        (
            "Each block is normalized by its best observed result among the eight policies.  "
            "Curves farther up and left are better; 1.00× means best or tied."
        ),
        ha="left",
        fontsize=9.25,
        color="#475569",
    )
    figure.subplots_adjust(top=0.87, bottom=0.14, left=0.09, right=0.98)
    paths = save_figure(figure, output_dir / "ao_arc_restart_policies_paired_cdf")
    plt.close(figure)
    return paths


def format_median(value: float) -> str:
    return f"{value:.0f}" if value.is_integer() else f"{value:.1f}"


def plot_anytime_facets(
    plt,
    traces: Sequence[TrialTrace],
    horizon: float,
    output_dir: Path,
) -> list[Path]:
    tasks = sorted({trace.task for trace in traces})
    figure, axes = plt.subplots(2, 3, figsize=(14.6, 9.0), dpi=180, sharex=True)
    figure.patch.set_facecolor("#F8FAFC")
    flat_axes = list(axes.flat)

    for panel, task in enumerate(tasks):
        axis = flat_axes[panel]
        task_traces = [trace for trace in traces if trace.task == task]
        endpoint_parts = []
        for policy in POLICIES:
            policy_traces = [trace for trace in task_traces if trace.policy == policy]
            grid = trace_grid(policy_traces, horizon)
            values = median_trace(policy_traces, grid, normalize=False)
            axis.step(
                grid,
                values,
                where="post",
                color=POLICY_COLORS[policy],
                linestyle=POLICY_LINESTYLES[policy],
                linewidth=2.05,
                label=POLICY_LABELS[policy],
            )
            endpoint_parts.append(
                f"{POLICY_SHORT_LABELS[policy]} {format_median(values[-1])}"
            )
        style_axis(axis)
        axis.set_title(f"Task {task}", loc="left", fontweight="bold")
        axis.set_ylabel("Median makespan (timesteps)")
        axis.text(
            0.97,
            0.94,
            endpoint_annotation(endpoint_parts),
            transform=axis.transAxes,
            ha="right",
            va="top",
            fontsize=6.9,
            family="DejaVu Sans Mono",
            color="#475569",
            bbox={"boxstyle": "round,pad=0.35", "facecolor": "#FFFFFF", "edgecolor": "#E2E8F0", "alpha": 0.92},
        )
        axis.margins(y=0.12)

    normalized_axis = flat_axes[len(tasks)]
    endpoint_parts = []
    for policy in POLICIES:
        policy_traces = [trace for trace in traces if trace.policy == policy]
        grid = trace_grid(policy_traces, horizon)
        values = [100.0 * value for value in median_trace(policy_traces, grid, normalize=True)]
        normalized_axis.step(
            grid,
            values,
            where="post",
            color=POLICY_COLORS[policy],
            linestyle=POLICY_LINESTYLES[policy],
            linewidth=2.15,
            label=POLICY_LABELS[policy],
        )
        endpoint_parts.append(f"{POLICY_SHORT_LABELS[policy]} {values[-1]:.1f}%")
    style_axis(normalized_axis)
    normalized_axis.set_title("All tasks · normalized", loc="left", fontweight="bold")
    normalized_axis.set_ylabel("Median incumbent / initial")
    normalized_axis.yaxis.set_major_formatter(lambda value, _position: f"{value:.0f}%")
    normalized_axis.text(
        0.97,
        0.94,
        endpoint_annotation(endpoint_parts),
        transform=normalized_axis.transAxes,
        ha="right",
        va="top",
        fontsize=6.9,
        family="DejaVu Sans Mono",
        color="#475569",
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "#FFFFFF", "edgecolor": "#E2E8F0", "alpha": 0.92},
    )
    normalized_axis.margins(y=0.12)

    xticks = [0.0, 0.1, 0.3, 1.0, 3.0, 10.0, 30.0, horizon]
    xticklabels = ["0", "0.1", "0.3", "1", "3", "10", "30", f"{horizon:.1f}"]
    for index, axis in enumerate(flat_axes):
        axis.set_xscale("symlog", linthresh=0.25, linscale=0.85, base=10)
        axis.set_xlim(0.0, horizon)
        axis.set_xticks(xticks)
        axis.set_xticklabels(xticklabels)
        axis.axvline(30.0, color="#94A3B8", linewidth=0.9, linestyle=":")
        if index >= 3:
            axis.set_xlabel("Runtime since first incumbent (s)")

    handles, labels = flat_axes[0].get_legend_handles_labels()
    # Matplotlib fills multi-column legends down each column. Keep related
    # baseline/random pairs together despite the analysis-oriented policy order.
    legend_order = (0, 1, 2, 3, 4, 6, 5, 7)
    handles = [handles[index] for index in legend_order]
    labels = [labels[index] for index in legend_order]
    figure.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.895),
        ncol=4,
        fontsize=8.6,
        frameon=True,
        facecolor="#FFFFFF",
        framealpha=0.96,
    )
    figure.suptitle(
        "Median AO-ARC makespan over optimization runtime",
        x=0.06,
        y=0.985,
        ha="left",
        fontsize=17,
        fontweight="bold",
        color="#0F172A",
    )
    figure.text(
        0.06,
        0.943,
        (
            "Panda Cage n=4  •  traces aligned to their first incumbent  •  "
            "task panels: 10 seeds  •  normalized panel: 50 matched blocks"
        ),
        ha="left",
        fontsize=10.3,
        color="#475569",
    )
    figure.text(
        0.06,
        0.018,
        (
            f"The common {horizon:.2f} s horizon is the shortest recorded post-incumbent runtime.  "
            "The dotted line marks 30 s.  The log-like time axis expands early progress; lower is better."
        ),
        ha="left",
        fontsize=9.2,
        color="#475569",
    )
    figure.subplots_adjust(top=0.78, bottom=0.11, left=0.07, right=0.985, wspace=0.25, hspace=0.34)
    paths = save_figure(
        figure,
        output_dir / "ao_arc_restart_policies_median_makespan_over_runtime",
    )
    plt.close(figure)
    return paths


def build_summary(
    traces: Sequence[TrialTrace],
    blocks: dict[tuple[int, int], dict[str, TrialTrace]],
    ratios: dict[str, list[float]],
    horizon: float,
) -> dict[str, object]:
    ratio_summary = {}
    for policy in POLICIES:
        values = ratios[policy]
        ratio_summary[policy] = {
            "best_or_tied_count": sum(
                math.isclose(value, 1.0, abs_tol=1e-12) for value in values
            ),
            "median_ratio_to_matched_best": statistics.median(values),
            "worst_ratio_to_matched_best": max(values),
            "within_threshold_count": {
                f"{threshold:g}": sum(value <= threshold + 1e-12 for value in values)
                for threshold in RATIO_THRESHOLDS
            },
        }

    checkpoints = [value for value in (0.0, 1.0, 5.0, 10.0, 20.0, 30.0, 60.0, horizon) if value <= horizon]
    anytime_summary = {}
    for policy in POLICIES:
        policy_traces = [trace for trace in traces if trace.policy == policy]
        anytime_summary[policy] = {
            f"{checkpoint:.12g}": {
                "median_makespan_timesteps": statistics.median(
                    trace.makespan_at(checkpoint) for trace in policy_traces
                ),
                "median_fraction_of_initial": statistics.median(
                    trace.makespan_at(checkpoint) / trace.initial_makespan
                    for trace in policy_traces
                ),
            }
            for checkpoint in checkpoints
        }

    return {
        "schema": "comotion.ao_arc_restart_policy_plots.v3",
        "paired_blocks": len(blocks),
        "trial_count": len(traces),
        "tasks": sorted({trace.task for trace in traces}),
        "seeds": sorted({trace.seed for trace in traces}),
        "policies": list(POLICIES),
        "paired_cdf_definition": (
            "final makespan divided by the best final makespan among the eight "
            "policies in the same task/seed block"
        ),
        "anytime_definition": (
            "right-continuous incumbent trace aligned to its first solution; "
            "pointwise median over a fixed matched cohort"
        ),
        "common_optimization_horizon_seconds": horizon,
        "ratio_summary": ratio_summary,
        "anytime_summary_all_tasks": anytime_summary,
    }


def main() -> int:
    args = parse_args()
    result_root = args.result_root.resolve()
    output_dir = (args.output_dir or (result_root / "plots")).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    traces = load_traces(result_root)
    blocks = group_by_block(traces)
    ratio_rows, ratios = paired_ratio_rows(blocks)
    common_horizon = min(trace.optimization_runtime for trace in traces)
    runtime_data, _plot_data = runtime_rows(traces, common_horizon)

    ratio_csv = output_dir / "ao_arc_restart_policies_paired_final_makespan.csv"
    ratio_columns = [
        "task",
        "seed",
        "initial_makespan_timesteps",
        "best_final_makespan_timesteps",
    ]
    for policy in POLICIES:
        ratio_columns.extend(
            [
                f"{policy}_final_makespan_timesteps",
                f"{policy}_ratio_to_matched_best",
            ]
        )
    write_csv(ratio_csv, ratio_columns, ratio_rows)

    runtime_csv = output_dir / "ao_arc_restart_policies_median_makespan_over_runtime.csv"
    write_csv(
        runtime_csv,
        [
            "scope",
            "elapsed_since_initial_seconds",
            "policy",
            "median_makespan_timesteps",
            "median_fraction_of_initial",
            "num_traces",
        ],
        runtime_data,
    )

    summary = build_summary(traces, blocks, ratios, common_horizon)
    summary_path = output_dir / "ao_arc_restart_policies_plot_summary.json"
    with summary_path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2, sort_keys=True)
        handle.write("\n")

    with tempfile.TemporaryDirectory(prefix="comotion-matplotlib-") as cache_dir:
        plt = configure_matplotlib(cache_dir)
        plot_paths = [
            *plot_paired_cdf(plt, ratios, output_dir),
            *plot_anytime_facets(plt, traces, common_horizon, output_dir),
        ]

    print(f"trials: {len(traces)}")
    print(f"paired_blocks: {len(blocks)}")
    print(f"common_optimization_horizon_seconds: {common_horizon:.9f}")
    print(f"paired_data: {ratio_csv}")
    print(f"anytime_data: {runtime_csv}")
    print(f"summary: {summary_path}")
    for path in plot_paths:
        print(f"plot: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
