#!/usr/bin/env python3
"""Verify completeness and integrity of the parc_final_8_20 campaign."""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_ROOT = REPO_ROOT / "benchmarks" / "results" / "parc_final_8_20"
SEEDS_2D = 30
TRIALS_PANDA = 50
SCALING_METHODS = ("ARC", "P-ARC-2", "P-ARC-4", "P-ARC-8", "P-ARC-16")
MAIN_METHODS = (
    "ARC", "P-ARC-16", "OR-ARC-16", "OR-P-ARC-4x4",
    "EP-RRT-C-16", "OR-RRT-C-16", "OR-EP-RRT-C-4x4",
    "OR-PP-ST-RRT-16",
)
PANDA_METHODS = (
    "ARC", "P-ARC-16", "OR-ARC-16", "OR-P-ARC-2x8",
    "OR-P-ARC-4x4", "OR-P-ARC-8x2", "EP-RRT-C-16",
    "OR-RRT-C-16", "OR-EP-RRT-C-4x4", "OR-PP-ST-RRT-16",
)
MAIN_CASES = {
    "mobile_parallel": (16, 64, 256),
    "mobile_circle": (4, 8, 16),
    "planar_cross": (16, 64, 256),
}
SCALING_CAPS = {"mobile_parallel": 256, "mobile_circle": 16, "planar_cross": 256}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--allow-incomplete", action="store_true")
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise RuntimeError(f"missing {path}")
    data = json.loads(path.read_text())
    if not isinstance(data, dict):
        raise RuntimeError(f"not a JSON object: {path}")
    return data


def load_rows(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        raise RuntimeError(f"missing {path}")
    with path.open(newline="") as handle:
        return [dict(row) for row in csv.DictReader(handle)]


def success(row: dict[str, str]) -> bool:
    return row.get("success", "").lower() in ("true", "1", "yes")


def grouped(rows: list[dict[str, str]]) -> dict[tuple[str, str], list[dict[str, str]]]:
    output: dict[tuple[str, str], list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        output[(row.get("case", ""), row.get("method", ""))].append(row)
    return output


def check_trial_rows(
    rows: list[dict[str, str]], *, expected_timeout: float,
    expected_assignment: str | None, require_repair_or: bool,
    errors: list[str], label: str,
) -> None:
    seen: set[tuple[str, str, str, str]] = set()
    for row in rows:
        identity = (
            row.get("case", ""), row.get("task_index", ""),
            row.get("seed", ""), row.get("method", ""),
        )
        if identity in seen:
            errors.append(f"{label}: duplicate trial {identity}")
        seen.add(identity)
        try:
            timeout = float(row.get("time_limit_seconds", "nan"))
        except ValueError:
            timeout = float("nan")
        if timeout != expected_timeout:
            errors.append(f"{label}: wrong timeout for {identity}: {timeout}")
        metrics_path = Path(row.get("metrics_json", ""))
        if not metrics_path.is_file():
            errors.append(f"{label}: missing metrics for {identity}: {metrics_path}")
            continue
        metrics = load_json(metrics_path)
        calls = metrics.get("validation_timing", {}).get("total_validation_calls")
        if calls != 0:
            errors.append(f"{label}: instrumentation active for {identity}: {calls}")
        method = row.get("method", "")
        if "P-ARC" not in method:
            continue
        stats = metrics.get("planner_stats", {})
        if expected_assignment is not None:
            actual = stats.get("parallel_arc_conflict_find_assignment")
            # OR-P-ARC can time out with no winning outer-OR worker.  In that
            # case there is no child planner_stats document to propagate into
            # the aggregate metrics, although the worker outcomes are still
            # present.  The campaign configuration is checked separately and
            # pins the assignment used to launch every worker, so only exempt
            # this narrowly identifiable no-winner timeout representation.
            outer_or = stats.get("or_parallel", {})
            no_winner_timeout = (
                method.startswith("OR-P-ARC")
                and not success(row)
                and metrics.get("planner_status") == "Timeout"
                and isinstance(outer_or, dict)
                and outer_or.get("winner_index") == -1
            )
            if actual != expected_assignment and not (
                actual is None and no_winner_timeout
            ):
                errors.append(
                    f"{label}: {identity} assignment={actual}, expected={expected_assignment}"
                )
        if require_repair_or and stats.get("parallel_arc_repair_duplicate_attempts") is not True:
            errors.append(f"{label}: local repair OR is off for {identity}")


def check_scaling(root: Path, errors: list[str]) -> list[dict[str, str]]:
    rows = load_rows(root / "p_arc_scaling.csv")
    check_trial_rows(
        rows, expected_timeout=30.0, expected_assignment="cyclic_cover_greedy",
        require_repair_or=False, errors=errors, label="scaling",
    )
    groups = grouped(rows)
    progress = load_json(root / "progress.json")
    decisions = progress.get("scaling_decisions", [])
    by_scenario: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for decision in decisions:
        if isinstance(decision, dict):
            by_scenario[str(decision.get("scenario", ""))].append(decision)
    for scenario, cap in SCALING_CAPS.items():
        scenario_decisions = by_scenario.get(scenario, [])
        if not scenario_decisions:
            errors.append(f"scaling: no completed sizes for {scenario}")
            continue
        sizes = [int(d.get("team_size", 0)) for d in scenario_decisions]
        expected_sizes = [4]
        while len(expected_sizes) < len(sizes):
            expected_sizes.append(expected_sizes[-1] * 2)
        if sizes != expected_sizes:
            errors.append(f"scaling: non-doubling sizes for {scenario}: {sizes}")
        active_parallel = set(SCALING_METHODS[1:])
        for size in sizes:
            case = f"{scenario}_n{size}"
            for method in SCALING_METHODS:
                count = len(groups.get((case, method), []))
                expected = (
                    SEEDS_2D
                    if method == "ARC" or method in active_parallel
                    else 0
                )
                if count != expected:
                    errors.append(
                        f"scaling: {case}/{method} has {count}, expected {expected}"
                    )
            for method in tuple(active_parallel):
                method_rows = groups.get((case, method), [])
                if len(method_rows) == SEEDS_2D and not any(
                    success(row) for row in method_rows
                ):
                    active_parallel.remove(method)
        last = scenario_decisions[-1]
        arc_successes = int(last.get("arc_successes", -1))
        should_stop = arc_successes <= 15 or sizes[-1] >= cap
        if not should_stop or not bool(last.get("stop_after_this_size")):
            errors.append(
                f"scaling: {scenario} ended improperly at n={sizes[-1]}, ARC={arc_successes}"
            )
    return rows


def check_pruned_matrix(
    rows: list[dict[str, str]], cases: dict[str, tuple[int, ...]],
    methods: tuple[str, ...], trials: int, errors: list[str], label: str,
) -> None:
    groups = grouped(rows)
    for scenario, sizes in cases.items():
        active = set(methods)
        for size in sizes:
            case = f"{scenario}_n{size}"
            for method in methods:
                count = len(groups.get((case, method), []))
                expected = trials if method in active else 0
                if count != expected:
                    errors.append(
                        f"{label}: {case}/{method} has {count}, expected {expected}"
                    )
            for method in tuple(active):
                method_rows = groups.get((case, method), [])
                if len(method_rows) == trials and not any(success(row) for row in method_rows):
                    active.remove(method)


def main() -> int:
    args = parse_args()
    root = args.root.resolve()
    errors: list[str] = []
    try:
        reuse = load_json(root / "reuse_manifest.json")
        if reuse.get("schema") != "comotion.parc_final_recovery.v1":
            errors.append("reuse manifest schema mismatch")

        config_2d = load_json(root / "experiment_config.json")
        if config_2d.get("parallel_arc_conflict_find_assignment") != "cyclic_cover_greedy":
            errors.append("2D campaign configuration does not pin cyclic_cover_greedy")
        variant_commands = config_2d.get("variant_commands_by_scenario", {})
        for scenario in MAIN_CASES:
            commands = variant_commands.get(scenario, {})
            for method in ("P-ARC-2", "P-ARC-4", "P-ARC-8", "P-ARC-16", "OR-P-ARC-4x4"):
                variant_args = commands.get(method, [])
                expected_pair = [
                    "--parallel-arc-conflict-find-assignment",
                    "cyclic_cover_greedy",
                ]
                if not any(
                    variant_args[index:index + 2] == expected_pair
                    for index in range(max(0, len(variant_args) - 1))
                ):
                    errors.append(
                        f"2D campaign configuration omits cyclic assignment for "
                        f"{scenario}/{method}"
                    )

        check_scaling(root, errors)

        main_rows = load_rows(root / "main_methods.csv")
        check_trial_rows(
            main_rows, expected_timeout=30.0,
            expected_assignment="cyclic_cover_greedy", require_repair_or=False,
            errors=errors, label="main 2D",
        )
        check_pruned_matrix(
            main_rows, MAIN_CASES, MAIN_METHODS, SEEDS_2D, errors, "main 2D"
        )

        panda_rows = load_rows(root / "panda" / "results.csv")
        panda_config = load_json(root / "panda" / "experiment_config.json")
        if panda_config.get("parallel_arc_conflict_find_assignment") != "cyclic_cover_greedy":
            errors.append("Panda campaign configuration does not pin cyclic_cover_greedy")
        check_trial_rows(
            panda_rows, expected_timeout=100.0,
            expected_assignment="cyclic_cover_greedy", require_repair_or=True,
            errors=errors, label="Panda",
        )
        check_pruned_matrix(
            panda_rows, {"panda_cage": (4, 8, 16)}, PANDA_METHODS,
            TRIALS_PANDA, errors, "Panda",
        )

        independence_path = root / "independence_ablation" / "results.csv"
        if independence_path.is_file():
            independence = load_rows(independence_path)
            if len(independence) != 100:
                errors.append(
                    f"independence ablation has {len(independence)}/100 rows"
                )
            check_trial_rows(
                independence, expected_timeout=100.0,
                expected_assignment="cyclic_cover_greedy",
                require_repair_or=True, errors=errors,
                label="independence ablation",
            )
        else:
            errors.append(f"missing {independence_path}")

        horizon_path = (
            root / "synchronization_horizon_ablation" / "results.csv"
        )
        if horizon_path.is_file():
            horizon = load_rows(horizon_path)
            if len(horizon) != 350:
                errors.append(f"horizon ablation has {len(horizon)}/350 rows")
            check_trial_rows(
                horizon, expected_timeout=100.0,
                expected_assignment="round_robin", require_repair_or=True,
                errors=errors, label="horizon ablation",
            )
        else:
            errors.append(f"missing {horizon_path}")

        log = root / "campaign.log"
        if not log.is_file() or "CAMPAIGN COMPLETE" not in log.read_text():
            errors.append("campaign completion marker missing")
    except RuntimeError as exc:
        errors.append(str(exc))

    if errors:
        print(f"verification errors: {len(errors)}")
        for error in errors:
            print(f"- {error}")
        return 0 if args.allow_incomplete else 1
    print(f"verified complete: {root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
