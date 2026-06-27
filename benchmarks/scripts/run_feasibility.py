#!/usr/bin/env python3
"""Run CoMotion feasibility benchmarks and plot cumulative success over runtime."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.dont_write_bytecode = True

from benchmark_runner_common import (
    DEFAULT_BUILD_DIR,
    DEFAULT_FEASIBILITY_CASES,
    DEFAULT_RESULTS_DIR,
    build_trial_specs,
    finish_outputs,
    parse_cases,
    parse_int_csv,
    run_trials,
    timestamp,
    variants_from_algorithms,
    write_manifest,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run feasibility trials through the benchmark executables. "
            "Outputs results.csv, solution_events.csv, and one "
            "cumulative-success plot per benchmark case."
        )
    )
    parser.add_argument(
        "--cases",
        default="default",
        help=(
            "Comma-separated benchmark cases, or 'default'. "
            "See benchmark_runner_common.py for the public case catalog."
        ),
    )
    parser.add_argument(
        "--algorithms",
        default="arc,prioritized,drrt,stcbs",
        help="Comma-separated planner algorithms to compare.",
    )
    parser.add_argument(
        "--parallel-arc-initial-solution-or",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Enable duplicate OR attempts for ParallelARC initial individual solutions.",
    )
    parser.add_argument(
        "--seeds",
        default="0",
        help="Comma-separated seed list. Use --num-seeds for 0..N-1.",
    )
    parser.add_argument(
        "--num-seeds",
        type=int,
        help="Use seeds 0..N-1 instead of --seeds.",
    )
    parser.add_argument(
        "--task-indices",
        default="0",
        help="Comma-separated task indices for task-based cases such as Panda.",
    )
    parser.add_argument("--time-limit", type=float, default=60.0)
    parser.add_argument("--collision-backend", default="vamp")
    parser.add_argument("--resolution", type=int, default=128)
    parser.add_argument("--build-dir", type=Path, default=DEFAULT_BUILD_DIR)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=None,
        help="Output directory. Default: benchmarks/results/feasibility_<UTC timestamp>.",
    )
    parser.add_argument("--jobs", type=int, default=1)
    parser.add_argument(
        "--timeout-grace",
        type=float,
        default=30.0,
        help="Seconds added to --time-limit for the process timeout.",
    )
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_root = args.output_root or DEFAULT_RESULTS_DIR / f"feasibility_{timestamp()}"
    cases = parse_cases(args.cases, DEFAULT_FEASIBILITY_CASES)
    extra_args: dict[str, tuple[str, ...]] = {}
    if args.parallel_arc_initial_solution_or:
        extra_args["parallel_arc"] = ("--parallel-arc-initial-solution-or",)
    variants = variants_from_algorithms(
        args.algorithms,
        extra_args_by_algorithm=extra_args,
    )
    seeds = list(range(args.num_seeds)) if args.num_seeds is not None else parse_int_csv(args.seeds)
    task_indices = parse_int_csv(args.task_indices)
    specs = build_trial_specs(
        cases=cases,
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
        return 0

    write_manifest(
        output_root,
        experiment_type="feasibility",
        command_line=sys.argv,
        cases=cases,
        variants=variants,
        trial_count=len(specs),
    )

    rows, event_rows = run_trials(
        specs,
        jobs=args.jobs,
        timeout_seconds=args.time_limit + args.timeout_grace,
    )
    plots = finish_outputs(
        output_root=output_root,
        result_rows=rows,
        event_rows=event_rows,
        plot_kind="success",
    )
    print(f"results_csv: {output_root / 'results.csv'}")
    print(f"solution_events_csv: {output_root / 'solution_events.csv'}")
    for plot in plots:
        print(f"plot: {plot}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
