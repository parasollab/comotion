#!/usr/bin/env python3
"""Run the Panda Cage optimistic-conflict P-ARC ablation."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import sys
from pathlib import Path
from typing import Any, Sequence

sys.dont_write_bytecode = True

from benchmark_runner_common import (
    CASE_CATALOG,
    DEFAULT_BUILD_DIR,
    DEFAULT_RESULTS_DIR,
    build_trial_specs,
    finish_outputs,
    paper_optimistic_conflict_ablation_variants,
    parse_int_csv,
    run_trials,
    timestamp,
    write_manifest,
)


SUMMARY_COLUMNS = [
    "case",
    "method",
    "conflict_batch_mode",
    "trial_set",
    "count",
    "runtime_seconds_mean",
    "conflict_detection_seconds_mean",
    "average_batch_size",
    "average_first_batch_size",
]

PAPER_TASK_INDICES = (0, 1, 2, 3, 4)
PAPER_SEED_COUNT = 10
PAPER_TIME_LIMIT_SECONDS = 100.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run full P-ARC planning trials on Panda Cage with 8 robots, "
            "comparing optimistic conflict batches against independent-only "
            "batches while preserving conflict-detection timing data."
        )
    )
    parser.add_argument("--num-seeds", type=int, default=10)
    parser.add_argument("--seeds", default=None)
    parser.add_argument("--task-indices", default="0,1,2,3,4")
    parser.add_argument("--time-limit", type=float, default=100.0)
    parser.add_argument("--collision-backend", default="vamp")
    parser.add_argument("--resolution", type=int, default=128)
    parser.add_argument("--build-dir", type=Path, default=DEFAULT_BUILD_DIR)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=None,
        help=(
            "Output directory. Default: benchmarks/results/"
            "parallel_arc_optimistic_ablation_<UTC timestamp>."
        ),
    )
    parser.add_argument(
        "--overwrite-results",
        action="store_true",
        help="Discard existing CSV rows and rerun requested trials.",
    )
    parser.add_argument("--jobs", type=int, default=1)
    parser.add_argument("--timeout-grace", type=float, default=30.0)
    parser.add_argument(
        "--allow-nonpaper-matrix",
        action="store_true",
        help=(
            "Allow task, seed-count, or timeout settings that do not match "
            "parallel_arc.pdf Table III."
        ),
    )
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def finite_float(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float, str)):
        return None
    try:
        number = float(value)
    except ValueError:
        return None
    return number


def mean_or_blank(values: Sequence[float]) -> float | str:
    return statistics.mean(values) if values else ""


def load_metrics(path: str) -> dict[str, Any]:
    if not path:
        return {}
    metrics_path = Path(path)
    if not metrics_path.is_file():
        return {}
    with metrics_path.open() as handle:
        data = json.load(handle)
    return data if isinstance(data, dict) else {}


def planner_stats(metrics: dict[str, Any]) -> dict[str, Any]:
    stats = metrics.get("planner_stats")
    return stats if isinstance(stats, dict) else {}


def conflict_round_batch_sizes(stats: dict[str, Any]) -> list[float]:
    rounds = stats.get("parallel_arc_conflict_rounds")
    if not isinstance(rounds, list):
        return []
    counts: list[float] = []
    for round_stats in rounds:
        if not isinstance(round_stats, dict):
            continue
        count = finite_float(round_stats.get("entry_count"))
        if count is not None:
            counts.append(count)
    return counts


def is_success(row: dict[str, Any]) -> bool:
    return str(row.get("success", "")).lower() == "true"


def paper_runtime_seconds(
    row: dict[str, Any], metrics: dict[str, Any]
) -> float | None:
    if not is_success(row):
        return finite_float(row.get("time_limit_seconds"))
    return finite_float(metrics.get("planning_time_seconds"))


def build_summary_rows(rows: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for row in rows:
        key = (str(row.get("case", "")), str(row.get("method", "")))
        grouped.setdefault(key, []).append(row)

    summary_rows: list[dict[str, Any]] = []
    for (case, method), method_rows in sorted(grouped.items()):
        for trial_set, selected_rows in (
            (
                "successful_trials",
                [row for row in method_rows if is_success(row)],
            ),
            ("all_trials", list(method_rows)),
        ):
            modes: list[str] = []
            runtimes: list[float] = []
            detection_wall: list[float] = []
            batch_sizes: list[float] = []
            first_batch_sizes: list[float] = []
            for row in selected_rows:
                metrics = load_metrics(str(row.get("metrics_json", "")))
                stats = planner_stats(metrics)
                mode = stats.get("parallel_arc_conflict_batch_mode")
                if isinstance(mode, str):
                    modes.append(mode)
                runtime = paper_runtime_seconds(row, metrics)
                if runtime is not None:
                    runtimes.append(runtime)
                detection = finite_float(
                    stats.get("conflict_detection_times_seconds_wall_clock")
                )
                if detection is not None:
                    detection_wall.append(detection)
                trial_batch_sizes = conflict_round_batch_sizes(stats)
                batch_sizes.extend(trial_batch_sizes)
                if trial_batch_sizes:
                    first_batch_sizes.append(trial_batch_sizes[0])

            summary_rows.append(
                {
                    "case": case,
                    "method": method,
                    "conflict_batch_mode": ",".join(sorted(set(modes))),
                    "trial_set": trial_set,
                    "count": len(selected_rows),
                    "runtime_seconds_mean": mean_or_blank(runtimes),
                    "conflict_detection_seconds_mean": mean_or_blank(
                        detection_wall
                    ),
                    "average_batch_size": mean_or_blank(batch_sizes),
                    "average_first_batch_size": mean_or_blank(
                        first_batch_sizes
                    ),
                }
            )
    return summary_rows


def validate_complete_matrix(
    rows: Sequence[dict[str, Any]],
    expected_methods: Sequence[str],
    seeds: Sequence[int],
    task_indices: Sequence[int],
) -> None:
    expected = {
        (method, task_index, seed)
        for method in expected_methods
        for task_index in task_indices
        for seed in seeds
    }
    actual_counts: dict[tuple[str, int, int], int] = {}
    invalid_rows: list[str] = []
    for row in rows:
        try:
            key = (
                str(row.get("method", "")),
                int(str(row.get("task_index", ""))),
                int(str(row.get("seed", ""))),
            )
        except ValueError:
            invalid_rows.append(
                f"{row.get('method', '')}/task={row.get('task_index', '')}"
                f"/seed={row.get('seed', '')}"
            )
            continue
        actual_counts[key] = actual_counts.get(key, 0) + 1

    actual = set(actual_counts)
    missing = sorted(expected - actual)
    unexpected = sorted(actual - expected)
    duplicates = sorted(
        (key, count) for key, count in actual_counts.items() if count != 1
    )
    if invalid_rows or missing or unexpected or duplicates:
        details = [
            f"missing={len(missing)}",
            f"unexpected={len(unexpected)}",
            f"duplicates={len(duplicates)}",
            f"invalid={len(invalid_rows)}",
        ]
        raise RuntimeError(
            "Incomplete Table III trial matrix: " + ", ".join(details)
        )


def write_summary(path: Path, rows: Sequence[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=SUMMARY_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def main() -> int:
    args = parse_args()
    output_root = (
        args.output_root
        if args.output_root is not None
        else DEFAULT_RESULTS_DIR / f"parallel_arc_optimistic_ablation_{timestamp()}"
    )
    result_csv_path = output_root / "results.csv"
    event_csv_path = output_root / "solution_events.csv"
    summary_csv_path = output_root / "optimistic_conflict_ablation_summary.csv"

    seeds = (
        parse_int_csv(args.seeds)
        if args.seeds is not None
        else list(range(args.num_seeds))
    )
    task_indices = parse_int_csv(args.task_indices)
    if not args.allow_nonpaper_matrix:
        if tuple(task_indices) != PAPER_TASK_INDICES:
            raise RuntimeError(
                "Table III requires --task-indices 0,1,2,3,4; pass "
                "--allow-nonpaper-matrix for a partial diagnostic run"
            )
        if len(seeds) != PAPER_SEED_COUNT or len(set(seeds)) != PAPER_SEED_COUNT:
            raise RuntimeError(
                f"Table III requires {PAPER_SEED_COUNT} distinct seeds; pass "
                "--allow-nonpaper-matrix for a diagnostic run"
            )
        if args.time_limit != PAPER_TIME_LIMIT_SECONDS:
            raise RuntimeError(
                f"Table III requires a {PAPER_TIME_LIMIT_SECONDS:g}-second "
                "timeout; pass --allow-nonpaper-matrix for a diagnostic run"
            )
    case = CASE_CATALOG["panda_cage_n8"]
    variants = paper_optimistic_conflict_ablation_variants()
    specs = build_trial_specs(
        cases=[case],
        variants=variants,
        seeds=seeds,
        task_indices=task_indices,
        time_limit=args.time_limit,
        collision_backend=args.collision_backend,
        resolution=args.resolution,
        build_dir=args.build_dir,
        output_root=output_root,
    )

    if args.dry_run:
        for spec in specs:
            print(" ".join(spec.command()))
        print(f"planned_trials: {len(specs)}")
        print(f"output_root: {output_root}")
        print(f"results_csv: {result_csv_path}")
        print(f"solution_events_csv: {event_csv_path}")
        print(f"summary_csv: {summary_csv_path}")
        return 0

    write_manifest(
        output_root,
        experiment_type="parallel_arc_optimistic_conflict_ablation_panda_cage_n8",
        command_line=sys.argv,
        cases=[case],
        variants=variants,
        trial_count=len(specs),
    )
    rows, event_rows = run_trials(
        specs,
        jobs=args.jobs,
        timeout_seconds=args.time_limit + args.timeout_grace,
        keep_metrics_json=True,
        result_csv_path=result_csv_path,
        event_csv_path=event_csv_path,
        skip_existing=not args.overwrite_results,
    )
    plots = finish_outputs(
        output_root=output_root,
        result_csv_path=result_csv_path,
        event_csv_path=event_csv_path,
        result_rows=rows,
        event_rows=event_rows,
        plot_kind="success",
    )
    if not args.allow_nonpaper_matrix:
        validate_complete_matrix(
            rows,
            [variant.label for variant in variants],
            seeds,
            task_indices,
        )
    write_summary(summary_csv_path, build_summary_rows(rows))

    print(f"results_csv: {result_csv_path}")
    print(f"solution_events_csv: {event_csv_path}")
    print(f"summary_csv: {summary_csv_path}")
    for plot in plots:
        print(f"plot: {plot}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
