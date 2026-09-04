#!/usr/bin/env python3
"""Reproduce the complete experiment matrix from the AO-ARC paper."""

from __future__ import annotations

import argparse
import sys
from dataclasses import replace
from pathlib import Path
from typing import Sequence

sys.dont_write_bytecode = True

from benchmark_runner_common import (
    CASE_CATALOG,
    DEFAULT_BUILD_DIR,
    DEFAULT_RESULTS_DIR,
    BenchmarkCase,
    PlannerVariant,
    TrialSpec,
    build_trial_specs,
    finish_outputs,
    parse_cases,
    parse_csv_tokens,
    parse_int_csv,
    resolve_output_paths,
    run_trials,
    timestamp,
    write_manifest,
)


PAPER_2D_CASE_KEYS = (
    "mobile_parallel_n4",
    "mobile_parallel_n8",
    "mobile_parallel_n16",
    "mobile_circle_n4",
    "mobile_circle_n8",
    "mobile_circle_n16",
    "planar_cross_n4",
    "planar_cross_n8",
    "planar_cross_n16",
)
PAPER_PANDA_CASE_KEYS = ("panda_cage_n4", "panda_cage_n8")
PAPER_CASE_KEYS = (*PAPER_2D_CASE_KEYS, *PAPER_PANDA_CASE_KEYS)
PAPER_SEEDS = tuple(range(10))
PAPER_TASK_INDICES = tuple(range(5))
PAPER_2D_TIME_LIMIT_SECONDS = 300.0
PAPER_PANDA_TIME_LIMIT_SECONDS = 600.0


PAPER_VARIANTS: dict[str, PlannerVariant] = {
    "arc": PlannerVariant(
        "ARC",
        "arc",
        "arc",
        ("--arc-local-composite-use-makespan-metric",),
    ),
    "ao_arc": PlannerVariant(
        "AO-ARC",
        "ao_arc",
        "ao_arc",
        (
            "--arc-local-composite-use-makespan-metric",
            "--ao-arc-selective-replanning",
            "--ao-arc-selective-initial-conflict-scan",
            "--ao-arc-repair-history-replanning-depth",
            "0",
            "--ao-arc-random-full-restart-probability",
            "0",
        ),
    ),
    "comp_rrtc": PlannerVariant(
        "CompRRTC",
        "composite",
        "comp_rrtc",
        ("--composite-rrt-use-makespan-metric",),
    ),
    "comp_aorrtc": PlannerVariant(
        "CompAORRTC",
        "composite_aorrtc",
        "comp_aorrtc",
    ),
    "drrt_star": PlannerVariant(
        "dRRT*",
        "drrt_star",
        "drrt_star",
        ("--drrt-cost-metric", "makespan"),
    ),
    "pp_strrt_star": PlannerVariant(
        "PP-ST-RRT*",
        "prioritized",
        "pp_strrt_star",
        (
            "--strrt-shuffle-priority-order",
            "--strrt-return-first-solution",
            "0",
            "--strrt-rewiring",
            "knearest",
        ),
    ),
}


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--cases",
        default="default",
        help=(
            "Comma-separated subset of the 11 paper cases. The default is "
            "all nine 2D team-size cases and both Panda team sizes."
        ),
    )
    parser.add_argument(
        "--methods",
        default="all",
        help=(
            "Comma-separated method keys: "
            + ",".join(PAPER_VARIANTS)
            + ". Default: all."
        ),
    )
    parser.add_argument("--seeds", default=",".join(map(str, PAPER_SEEDS)))
    parser.add_argument("--num-seeds", type=int)
    parser.add_argument(
        "--task-indices",
        default=",".join(map(str, PAPER_TASK_INDICES)),
    )
    parser.add_argument(
        "--task-generation-seed",
        type=int,
        default=0,
        help=(
            "Fixed seed used to generate the five Panda tasks. It is kept "
            "separate from each trial's planning seed."
        ),
    )
    parser.add_argument(
        "--time-limit-2d", type=float, default=PAPER_2D_TIME_LIMIT_SECONDS
    )
    parser.add_argument(
        "--time-limit-panda",
        type=float,
        default=PAPER_PANDA_TIME_LIMIT_SECONDS,
    )
    parser.add_argument("--collision-backend", default="vamp")
    parser.add_argument("--resolution", type=int, default=128)
    parser.add_argument("--build-dir", type=Path, default=DEFAULT_BUILD_DIR)
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--results-csv", type=Path)
    parser.add_argument("--solution-events-csv", type=Path)
    parser.add_argument("--keep-metrics-json", action="store_true")
    parser.add_argument("--overwrite-results", action="store_true")
    parser.add_argument("--jobs", type=int, default=1)
    parser.add_argument("--timeout-grace", type=float, default=30.0)
    parser.add_argument("--plot-backends", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args(argv)


def paper_variants(value: str) -> list[PlannerVariant]:
    keys = list(PAPER_VARIANTS) if value == "all" else parse_csv_tokens(value)
    if not keys:
        raise RuntimeError("At least one paper method is required")
    try:
        return [PAPER_VARIANTS[key] for key in keys]
    except KeyError as exc:
        raise RuntimeError(
            f"Unknown paper method '{exc.args[0]}'. Available: "
            + ", ".join(PAPER_VARIANTS)
        ) from exc


def paper_cases(value: str, task_generation_seed: int) -> list[BenchmarkCase]:
    cases = parse_cases(value, PAPER_CASE_KEYS)
    unsupported = [case.key for case in cases if case.key not in PAPER_CASE_KEYS]
    if unsupported:
        raise RuntimeError(
            "Cases outside the AO-ARC paper matrix: " + ", ".join(unsupported)
        )
    return [
        replace(
            case,
            base_args=(
                *case.base_args,
                "--task-generation-seed",
                str(task_generation_seed),
            ),
        )
        if case.task_based
        else case
        for case in cases
    ]


def build_paper_specs(
    args: argparse.Namespace,
    *,
    output_root: Path,
) -> tuple[list[BenchmarkCase], list[PlannerVariant], list[TrialSpec]]:
    if args.num_seeds is not None and args.num_seeds < 1:
        raise RuntimeError("--num-seeds must be positive")
    if args.task_generation_seed < 0:
        raise RuntimeError("--task-generation-seed must be non-negative")
    if args.time_limit_2d <= 0.0 or args.time_limit_panda <= 0.0:
        raise RuntimeError("Paper time limits must be positive")
    if args.jobs < 1:
        raise RuntimeError("--jobs must be at least 1")
    if args.timeout_grace < 0.0:
        raise RuntimeError("--timeout-grace must be non-negative")

    cases = paper_cases(args.cases, args.task_generation_seed)
    variants = paper_variants(args.methods)
    seeds = (
        list(range(args.num_seeds))
        if args.num_seeds is not None
        else parse_int_csv(args.seeds)
    )
    task_indices = parse_int_csv(args.task_indices)
    if not seeds:
        raise RuntimeError("At least one planning seed is required")
    if not task_indices:
        raise RuntimeError("At least one Panda task index is required")
    if any(index not in PAPER_TASK_INDICES for index in task_indices):
        raise RuntimeError("AO-ARC paper task indices must be in [0, 4]")

    specs = []
    for case in cases:
        specs.extend(
            build_trial_specs(
                cases=[case],
                variants=variants,
                seeds=seeds,
                task_indices=task_indices,
                time_limit=(
                    args.time_limit_panda
                    if case.task_based
                    else args.time_limit_2d
                ),
                collision_backend=args.collision_backend,
                resolution=args.resolution,
                build_dir=args.build_dir,
                output_root=output_root,
            )
        )
    return cases, variants, specs


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    output_root, result_csv_path, event_csv_path = resolve_output_paths(
        output_root=args.output_root,
        results_csv=args.results_csv,
        solution_events_csv=args.solution_events_csv,
        default_output_root=DEFAULT_RESULTS_DIR / f"ao_arc_paper_{timestamp()}",
    )
    cases, variants, specs = build_paper_specs(args, output_root=output_root)

    if args.dry_run:
        for case in cases:
            for variant in variants:
                spec = next(
                    item
                    for item in specs
                    if item.case.key == case.key and item.variant == variant
                )
                print("COMMAND", " ".join(spec.command()))
        print(f"planned_trials: {len(specs)}")
        print(f"output_root: {output_root}")
        print(f"results_csv: {result_csv_path}")
        print(f"solution_events_csv: {event_csv_path}")
        return 0

    seeds = sorted({spec.seed for spec in specs})
    task_indices = sorted(
        {spec.task_index for spec in specs if spec.task_index is not None}
    )
    write_manifest(
        output_root,
        experiment_type="ao_arc_paper",
        command_line=sys.argv if argv is None else [sys.argv[0], *argv],
        cases=cases,
        variants=variants,
        trial_count=len(specs),
        settings={
            "paper": (
                "AO-ARC: Almost-Surely Asymptotically Optimal Multi-Robot "
                "Motion Planning with ARC"
            ),
            "seeds": seeds,
            "panda_task_indices": task_indices,
            "panda_task_generation_seed": args.task_generation_seed,
            "time_limit_2d_seconds": args.time_limit_2d,
            "time_limit_panda_seconds": args.time_limit_panda,
            "timeout_grace_seconds": args.timeout_grace,
            "collision_backend": args.collision_backend,
            "resolution": args.resolution,
            "reported_costs": (
                "raw makespan and sum of costs; no empirical J_hat "
                "normalization"
            ),
        },
    )

    rows, event_rows = run_trials(
        specs,
        jobs=args.jobs,
        timeout_seconds=None,
        timeout_grace_seconds=args.timeout_grace,
        keep_metrics_json=args.keep_metrics_json,
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
        plot_kind="anytime",
        plot_backends=args.plot_backends,
    )
    print(f"results_csv: {result_csv_path}")
    print(f"solution_events_csv: {event_csv_path}")
    for plot in plots:
        print(f"plot: {plot}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
