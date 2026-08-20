#!/usr/bin/env python3
"""Run the paper's Panda Cage conflict-horizon ablation."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.dont_write_bytecode = True

from benchmark_runner_common import (
    DEFAULT_BUILD_DIR,
    DEFAULT_RESULTS_DIR,
    PARALLEL_ARC_ASSIGNMENT_FLAG,
    PlannerVariant,
    build_trial_specs,
    finish_outputs,
    parse_cases,
    parse_int_csv,
    run_trials,
    timestamp,
    write_manifest,
)


DEFAULT_CASES = ("panda_cage_n8",)
PAPER_HORIZONS = (50, 100, 200, 400, 800, 1600, 3200)
PAPER_TASK_INDICES = (0, 1, 2, 3, 4)
PAPER_SEED_COUNT = 10
PAPER_TIME_LIMIT_SECONDS = 100.0

PANDA_ARC_PROFILE_ARGS = (
    "--arc-initial-window", "20",
    "--arc-expansion-step", "1.05",
    "--arc-expansion-policy", "exponential",
    "--arc-initial-valid-expansion-policy", "linear",
    "--arc-initial-valid-expansion-step", "20",
    "--arc-initial-valid-asymmetric-expansion",
    "--arc-cspace-bound-margin", "2",
    "--arc-min-cspace-bound-range", "2",
    "--arc-simplification-max-shortcut-steps", "128",
    "--arc-simplification-max-empty-steps", "32",
    "--arc-simplification-max-smooth-steps", "1",
    "--arc-simplification-max-passes", "1",
    "--arc-local-composite-max-samples", "250000",
    "--arc-local-composite-use-makespan-metric",
    "--arc-simplify-initial-solutions",
    "--no-arc-simplify-conflict-solutions",
    "--arc-local-solvers", "composite",
    "--arc-local-prioritized-max-iterations", "10",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Sweep round-robin conflict-pair detection horizons on a single "
            "ParallelARC conflict-detection call."
        )
    )
    parser.add_argument("--cases", default="default")
    parser.add_argument("--horizons", default="50,100,200,400,800,1600,3200")
    parser.add_argument("--num-seeds", type=int, default=10)
    parser.add_argument(
        "--task-indices",
        default="0,1,2,3,4",
        help="Task indices for task-based cases such as Panda Cage.",
    )
    parser.add_argument("--time-limit", type=float, default=100.0)
    parser.add_argument("--collision-backend", default="vamp")
    parser.add_argument("--resolution", type=int, default=128)
    parser.add_argument("--worker-processes", type=int, default=16)
    parser.add_argument("--build-dir", type=Path, default=DEFAULT_BUILD_DIR)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=None,
        help=(
            "Output directory. Default: benchmarks/results/"
            "parallel_arc_conflict_ablation_<UTC timestamp>."
        ),
    )
    parser.add_argument(
        "--overwrite-results",
        action="store_true",
        help="Discard existing CSV rows and rerun requested trials.",
    )
    parser.add_argument("--jobs", type=int, default=1)
    parser.add_argument("--timeout-grace", type=float, default=30.0)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def horizon_variants(
    horizons: list[int], worker_processes: int
) -> list[PlannerVariant]:
    return [
        PlannerVariant(
            label=f"P-ARC-{worker_processes}-h{horizon}",
            algorithm="parallel_arc",
            slug=f"p_arc_{worker_processes}_h{horizon}",
            extra_args=(
                "--parallel-arc-worker-processes",
                str(worker_processes),
                "--parallel-arc-parallel-initial-plans",
                "--parallel-arc-initial-solution-or",
                "--parallel-arc-repair-duplicate-attempts",
                "--parallel-arc-strategy", "synchronous",
                "--parallel-arc-conflict-strategy", "greedy",
                "--parallel-arc-conflict-find-mode", "segment_parallel",
                "--parallel-arc-conflict-batch-mode", "optimistic",
                PARALLEL_ARC_ASSIGNMENT_FLAG,
                "round_robin",
                "--parallel-arc-conflict-find-horizon",
                str(horizon),
                "--parallel-arc-conflict-ablation-only",
                *PANDA_ARC_PROFILE_ARGS,
            ),
        )
        for horizon in horizons
    ]


def main() -> int:
    args = parse_args()
    output_root = args.output_root or (
        DEFAULT_RESULTS_DIR / f"parallel_arc_conflict_ablation_{timestamp()}"
    )
    cases = parse_cases(args.cases, DEFAULT_CASES)
    horizons = parse_int_csv(args.horizons)
    task_indices = parse_int_csv(args.task_indices)
    if tuple(case.key for case in cases) != DEFAULT_CASES:
        raise RuntimeError("Table IV requires the panda_cage_n8 case")
    if tuple(horizons) != PAPER_HORIZONS:
        raise RuntimeError("Table IV requires horizons 50,100,200,400,800,1600,3200")
    if args.num_seeds != PAPER_SEED_COUNT:
        raise RuntimeError("Table IV requires 10 seeds")
    if tuple(task_indices) != PAPER_TASK_INDICES:
        raise RuntimeError("Table IV requires task indices 0,1,2,3,4")
    if args.time_limit != PAPER_TIME_LIMIT_SECONDS:
        raise RuntimeError("Table IV uses the 100-second Panda configuration")
    if args.jobs != 1:
        raise RuntimeError("The final campaign requires --jobs 1")
    variants = horizon_variants(horizons, args.worker_processes)
    specs = build_trial_specs(
        cases=cases,
        variants=variants,
        seeds=list(range(args.num_seeds)),
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
        return 0

    output_root.mkdir(parents=True, exist_ok=True)
    (output_root / "experiment_config.json").write_text(
        json.dumps(
            {
                "schema": "comotion.parallel_arc_horizon_ablation.v1",
                "case": "panda_cage_n8",
                "horizons": horizons,
                "seeds": list(range(args.num_seeds)),
                "task_indices": task_indices,
                "time_limit_seconds": args.time_limit,
                "timeout_grace_seconds": args.timeout_grace,
                "collision_backend": args.collision_backend,
                "resolution": args.resolution,
                "worker_processes": args.worker_processes,
                "top_level_trial_jobs": args.jobs,
                "validation_instrumentation": False,
                "conflict_find_assignment": "round_robin",
                "conflict_batch_mode": "optimistic",
                "conflict_ablation_only": True,
                "arc_profile_args": list(PANDA_ARC_PROFILE_ARGS),
            },
            indent=2,
        ) + "\n"
    )
    write_manifest(
        output_root,
        experiment_type="parallel_arc_conflict_horizon_ablation",
        command_line=sys.argv,
        cases=cases,
        variants=variants,
        trial_count=len(specs),
    )
    result_csv = output_root / "results.csv"
    event_csv = output_root / "solution_events.csv"
    rows, event_rows = run_trials(
        specs,
        jobs=args.jobs,
        timeout_seconds=args.time_limit + args.timeout_grace,
        keep_metrics_json=True,
        result_csv_path=result_csv,
        event_csv_path=event_csv,
        skip_existing=not args.overwrite_results,
    )
    finish_outputs(
        output_root=output_root,
        result_csv_path=result_csv,
        event_csv_path=event_csv,
        result_rows=rows,
        event_rows=event_rows,
        plot_kind="none",
    )
    print(f"output_root: {output_root}")
    print(f"results_csv: {result_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
