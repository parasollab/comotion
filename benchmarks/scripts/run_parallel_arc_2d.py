#!/usr/bin/env python3
"""Reproduce the final P-ARC 2D scaling and paper comparison experiments.

The worker-scaling phase runs first.  For each scenario, robot counts double
from four and the complete ARC/P-ARC worker matrix is run at every size.  The
first size where sequential ARC solves at most half of the 30 trials is
included, then that scenario stops scaling.  Scenario-specific hard caps are
also included and stop scaling.  The main eight-method paper comparison runs
only after all three scaling scenarios finish.

All top-level trials are serial.  Individual planners may use their declared
worker processes internally.  Results and per-trial metrics are resumable.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Sequence

sys.dont_write_bytecode = True

from benchmark_runner_common import (
    DEFAULT_BUILD_DIR,
    DEFAULT_PARALLEL_ARC_CONFLICT_FIND_ASSIGNMENT,
    DEFAULT_RESULTS_DIR,
    EVENT_COLUMNS,
    RESULT_COLUMNS,
    BenchmarkCase,
    PlannerVariant,
    build_trial_specs,
    effective_variant_extra_args,
    finish_outputs,
    load_csv_rows,
    run_trials,
    timestamp,
    truthy,
    write_csv,
)


SEEDS = tuple(range(30))
TIME_LIMIT_SECONDS = 30.0
TIMEOUT_GRACE_SECONDS = 30.0
COLLISION_BACKEND = "vamp"
RESOLUTION = 128
MAX_CORES = 16
ARC_SUCCESS_STOP_COUNT = len(SEEDS) // 2
DEFAULT_OUTPUT_ROOT = DEFAULT_RESULTS_DIR / f"parallel_arc_2d_{timestamp()}"

SCENARIOS = ("mobile_parallel", "mobile_circle", "planar_cross")
SCALING_CAPS = {
    "mobile_parallel": 256,
    "mobile_circle": 16,
    "planar_cross": 256,
}
MAIN_CASES: dict[str, tuple[int, ...]] = {
    "mobile_parallel": (16, 64, 256),
    "mobile_circle": (4, 8, 16),
    "planar_cross": (16, 64, 256),
}

# Mobile scenarios use the original paper's 200-timestep symmetric linear
# repair windows, with coupled/composite-only local repair.  Keep every option
# explicit so future executable-default changes cannot alter the experiment.
MOBILE_ARC_ARGS = (
    "--arc-initial-window",
    "200",
    "--arc-expansion-policy",
    "linear",
    "--arc-expansion-step",
    "200",
    "--arc-initial-valid-expansion-policy",
    "linear",
    "--arc-initial-valid-expansion-step",
    "200",
    "--arc-initial-valid-symmetric-expansion",
    "--arc-cspace-bound-margin",
    "2",
    "--arc-min-cspace-bound-range",
    "2",
    "--arc-local-composite-max-samples",
    "5000",
    "--arc-simplify-initial-solutions",
    "--no-arc-simplify-conflict-solutions",
    "--arc-local-solvers",
    "composite",
)

# Final Planar Cross profile selected for the P-ARC paper experiments.
PLANAR_ARC_ARGS = (
    "--arc-initial-window",
    "100",
    "--arc-expansion-policy",
    "exponential",
    "--arc-expansion-step",
    "1.05",
    "--arc-initial-valid-expansion-policy",
    "linear",
    "--arc-initial-valid-expansion-step",
    "10",
    "--arc-initial-valid-asymmetric-expansion",
    "--arc-cspace-bound-margin",
    "1",
    "--arc-min-cspace-bound-range",
    "0.5",
    "--arc-local-composite-max-samples",
    "50000",
    "--arc-simplify-initial-solutions",
    "--arc-simplify-conflict-solutions",
    "--arc-local-solvers",
    "composite",
)

# Local repair OR-multi-start is intentionally off.  Initial-solution OR is a
# separate P-ARC mechanism and remains enabled, matching the selected runs.
PARALLEL_ARC_ARGS = (
    "--parallel-arc-parallel-initial-plans",
    "--parallel-arc-initial-solution-or",
    "--no-parallel-arc-repair-duplicate-attempts",
    "--parallel-arc-strategy",
    "synchronous",
    "--parallel-arc-conflict-strategy",
    "greedy",
    "--parallel-arc-conflict-find-mode",
    "segment_parallel",
    "--parallel-arc-conflict-batch-mode",
    "optimistic",
    "--parallel-arc-conflict-find-horizon",
    "400",
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def phase(message: str) -> None:
    print(f"\n[{utc_now()}] {message}", flush=True)


def arc_args(scenario: str) -> tuple[str, ...]:
    if scenario == "planar_cross":
        return PLANAR_ARC_ARGS
    if scenario in ("mobile_parallel", "mobile_circle"):
        return MOBILE_ARC_ARGS
    raise RuntimeError(f"Unknown 2D scenario: {scenario}")


def benchmark_case(scenario: str, size: int) -> BenchmarkCase:
    if size < 2:
        raise RuntimeError("2D P-ARC cases require at least two robots")
    if scenario == "mobile_parallel":
        return BenchmarkCase(
            key=f"mobile_parallel_n{size}",
            title=f"Mobile Cross, {size} robots",
            executable="mobile_robot_2d_crossing",
            base_args=("--scenario", "parallel", "--num-robots", str(size)),
        )
    if scenario == "mobile_circle":
        return BenchmarkCase(
            key=f"mobile_circle_n{size}",
            title=f"Mobile Circle, {size} robots",
            executable="mobile_robot_2d_crossing",
            base_args=("--scenario", "circle", "--num-robots", str(size)),
        )
    if scenario == "planar_cross":
        return BenchmarkCase(
            key=f"planar_cross_n{size}",
            title=f"Planar Cross, {size} robots",
            executable="planar_manipulator_cross",
            base_args=("--scenario", "cross", "--num-robots", str(size)),
        )
    raise RuntimeError(f"Unknown 2D scenario: {scenario}")


def arc_variant(scenario: str) -> PlannerVariant:
    return PlannerVariant("ARC", "arc", "arc", arc_args(scenario))


def p_arc_variant(scenario: str, workers: int) -> PlannerVariant:
    return PlannerVariant(
        f"P-ARC-{workers}",
        "parallel_arc",
        f"p_arc_{workers}",
        (
            "--parallel-arc-worker-processes",
            str(workers),
            *PARALLEL_ARC_ARGS,
            *arc_args(scenario),
        ),
    )


def scaling_variants(scenario: str) -> list[PlannerVariant]:
    return [
        arc_variant(scenario),
        *(p_arc_variant(scenario, workers) for workers in (2, 4, 8, 16)),
    ]


def main_variants(scenario: str) -> list[PlannerVariant]:
    profile = arc_args(scenario)
    return [
        arc_variant(scenario),
        p_arc_variant(scenario, 16),
        PlannerVariant(
            "OR-ARC-16",
            "arc",
            "or_arc_16",
            ("--or-parallel-worker-processes", "16", *profile),
        ),
        PlannerVariant(
            "OR-P-ARC-4x4",
            "parallel_arc",
            "or_p_arc_4x4",
            (
                "--or-parallel-worker-processes",
                "4",
                "--parallel-arc-worker-processes",
                "4",
                *PARALLEL_ARC_ARGS,
                *profile,
            ),
        ),
        PlannerVariant(
            "EP-RRT-C-16",
            "cooperative_composite",
            "ep_rrt_c_16",
            ("--cooperative-rrt-worker-threads", "16"),
        ),
        PlannerVariant(
            "OR-RRT-C-16",
            "composite",
            "or_rrt_c_16",
            ("--or-parallel-worker-processes", "16"),
        ),
        PlannerVariant(
            "OR-EP-RRT-C-4x4",
            "cooperative_composite",
            "or_ep_rrt_c_4x4",
            (
                "--or-parallel-worker-processes",
                "4",
                "--cooperative-rrt-worker-threads",
                "4",
            ),
        ),
        PlannerVariant(
            "OR-PP-ST-RRT-16",
            "prioritized",
            "or_pp_st_rrt_16",
            (
                "--or-parallel-worker-processes",
                "16",
                "--strrt-shuffle-priority-order",
                "--strrt-rewiring",
                "off",
            ),
        ),
    ]


def result_rows_for(
    path: Path, *, case: str, method: str
) -> list[dict[str, str]]:
    return [
        row
        for row in load_csv_rows(path)
        if row.get("case") == case and row.get("method") == method
    ]


def success_count(path: Path, *, case: str, method: str) -> int:
    return sum(
        truthy(row.get("success"))
        for row in result_rows_for(path, case=case, method=method)
    )


def run_variant(
    *,
    case: BenchmarkCase,
    variant: PlannerVariant,
    output_root: Path,
    result_csv: Path,
    event_csv: Path,
    build_dir: Path,
) -> None:
    specs = build_trial_specs(
        cases=[case],
        variants=[variant],
        seeds=SEEDS,
        task_indices=[0],
        time_limit=TIME_LIMIT_SECONDS,
        collision_backend=COLLISION_BACKEND,
        resolution=RESOLUTION,
        build_dir=build_dir,
        output_root=output_root,
    )
    print(
        f"=== {case.key}/{variant.label}: jobs=1 trials={len(specs)} ===",
        flush=True,
    )
    run_trials(
        specs,
        jobs=1,
        timeout_seconds=TIME_LIMIT_SECONDS + TIMEOUT_GRACE_SECONDS,
        keep_metrics_json=True,
        result_csv_path=result_csv,
        event_csv_path=event_csv,
        skip_existing=True,
    )


def copy_scaling_overlap_to_main(
    *,
    scaling_results: Path,
    scaling_events: Path,
    main_results: Path,
    main_events: Path,
) -> None:
    overlap_cases = {
        benchmark_case(scenario, size).key
        for scenario, sizes in MAIN_CASES.items()
        for size in sizes
    }
    overlap_methods = {"ARC", "P-ARC-16"}
    selected_results = [
        row
        for row in load_csv_rows(scaling_results)
        if row.get("case") in overlap_cases
        and row.get("method") in overlap_methods
    ]
    selected_events = [
        row
        for row in load_csv_rows(scaling_events)
        if row.get("case") in overlap_cases
        and row.get("method") in overlap_methods
    ]

    result_replace_keys = {
        (
            row.get("case"),
            row.get("task_index"),
            row.get("seed"),
            row.get("method"),
        )
        for row in selected_results
    }
    event_replace_keys = {
        (
            row.get("case"),
            row.get("task_index"),
            row.get("seed"),
            row.get("method"),
            row.get("elapsed_seconds"),
            row.get("makespan_timesteps"),
        )
        for row in selected_events
    }
    retained_results = [
        row
        for row in load_csv_rows(main_results)
        if (
            row.get("case"),
            row.get("task_index"),
            row.get("seed"),
            row.get("method"),
        )
        not in result_replace_keys
    ]
    retained_events = [
        row
        for row in load_csv_rows(main_events)
        if (
            row.get("case"),
            row.get("task_index"),
            row.get("seed"),
            row.get("method"),
            row.get("elapsed_seconds"),
            row.get("makespan_timesteps"),
        )
        not in event_replace_keys
    ]
    write_csv(
        main_results,
        RESULT_COLUMNS,
        [*retained_results, *selected_results],
    )
    write_csv(
        main_events,
        EVENT_COLUMNS,
        [*retained_events, *selected_events],
    )


def write_json(path: Path, document: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(document, indent=2) + "\n")
    temporary.replace(path)


def profile_document(args: tuple[str, ...]) -> list[str]:
    return list(args)


def experiment_config() -> dict[str, object]:
    variant_profiles: dict[str, dict[str, list[str]]] = {}
    for scenario in SCENARIOS:
        variants = {
            variant.label: list(effective_variant_extra_args(variant))
            for variant in (
                *scaling_variants(scenario),
                *main_variants(scenario),
            )
        }
        variant_profiles[scenario] = variants
    return {
        "schema": "comotion.parallel_arc_2d.v1",
        "paper": "P-ARC: Exploiting Subproblem Independence for Parallel Multi-Robot Motion Planning",
        "paper_doi": "https://doi.org/10.48550/arXiv.2606.27625",
        "phase_order": ["p_arc_scaling", "main_2d_comparison"],
        "panda_experiments_included": False,
        "seeds": list(SEEDS),
        "time_limit_seconds": TIME_LIMIT_SECONDS,
        "timeout_grace_seconds": TIMEOUT_GRACE_SECONDS,
        "collision_backend": COLLISION_BACKEND,
        "resolution": RESOLUTION,
        "top_level_trial_jobs": 1,
        "max_internal_workers": MAX_CORES,
        "validation_instrumentation": False,
        "repair_seed_schedule": (
            "unique per outer trial, logical repair batch/task, and attempt; "
            "independent of worker slot"
        ),
        "scaling": {
            "scenarios": list(SCENARIOS),
            "initial_team_size": 4,
            "team_size_progression": "double after each completed size",
            "methods": ["ARC", "P-ARC-2", "P-ARC-4", "P-ARC-8", "P-ARC-16"],
            "stop_rule": (
                "include the first size where ARC has at most 15 successes "
                "among 30 trials, then stop that scenario; also stop after "
                "including the scenario-specific hard cap"
            ),
            "hard_caps": dict(SCALING_CAPS),
            "method_pruning": (
                "skip a P-ARC worker variant at larger sizes only after it "
                "has zero successes among all 30 trials"
            ),
        },
        "main_cases": {key: list(value) for key, value in MAIN_CASES.items()},
        "main_methods": [variant.label for variant in main_variants("mobile_parallel")],
        "arc_profiles": {
            "mobile_parallel": profile_document(MOBILE_ARC_ARGS),
            "mobile_circle": profile_document(MOBILE_ARC_ARGS),
            "planar_cross": profile_document(PLANAR_ARC_ARGS),
        },
        "parallel_arc_profile": profile_document(PARALLEL_ARC_ARGS),
        "parallel_arc_conflict_find_assignment": (
            DEFAULT_PARALLEL_ARC_CONFLICT_FIND_ASSIGNMENT
        ),
        "variant_commands_by_scenario": variant_profiles,
        "overlap_reuse": {
            "source": "p_arc_scaling.csv",
            "target": "main_methods.csv",
            "methods": ["ARC", "P-ARC-16"],
        },
        "main_method_pruning": (
            "after a method has zero successes at a team size, skip it at "
            "larger main-comparison sizes in that scenario"
        ),
        "trial_process_group_cleanup_required": True,
    }


def write_progress(
    path: Path,
    *,
    phase_name: str,
    scaling_decisions: Sequence[dict[str, object]],
    main_decisions: Sequence[dict[str, object]],
) -> None:
    write_json(
        path,
        {
            "schema": "comotion.parallel_arc_2d_progress.v1",
            "updated_utc": utc_now(),
            "current_phase": phase_name,
            "scaling_decisions": list(scaling_decisions),
            "main_pruning_decisions": list(main_decisions),
        },
    )


def run_scaling(
    *,
    output_root: Path,
    result_csv: Path,
    event_csv: Path,
    build_dir: Path,
    progress_path: Path,
) -> list[dict[str, object]]:
    decisions: list[dict[str, object]] = []
    for scenario in SCENARIOS:
        active_parallel = {
            variant.label
            for variant in scaling_variants(scenario)
            if variant.algorithm == "parallel_arc"
        }
        size = 4
        while True:
            case = benchmark_case(scenario, size)
            phase(f"P-ARC scaling: {case.key}")
            for variant in scaling_variants(scenario):
                if (
                    variant.algorithm == "parallel_arc"
                    and variant.label not in active_parallel
                ):
                    print(f"pruned: {case.key} {variant.label}", flush=True)
                    continue
                run_variant(
                    case=case,
                    variant=variant,
                    output_root=output_root,
                    result_csv=result_csv,
                    event_csv=event_csv,
                    build_dir=build_dir,
                )

            arc_rows = result_rows_for(
                result_csv, case=case.key, method="ARC"
            )
            arc_successes = success_count(
                result_csv, case=case.key, method="ARC"
            )
            if len(arc_rows) != len(SEEDS):
                raise RuntimeError(
                    f"{case.key} ARC has {len(arc_rows)} result rows; "
                    f"expected {len(SEEDS)}"
                )
            stop_for_success_rate = arc_successes <= ARC_SUCCESS_STOP_COUNT
            stop_for_cap = size >= SCALING_CAPS[scenario]
            stop = stop_for_success_rate or stop_for_cap
            parallel_results: list[dict[str, object]] = []
            for variant in scaling_variants(scenario):
                if (
                    variant.algorithm != "parallel_arc"
                    or variant.label not in active_parallel
                ):
                    continue
                rows = result_rows_for(
                    result_csv, case=case.key, method=variant.label
                )
                successes = success_count(
                    result_csv, case=case.key, method=variant.label
                )
                if len(rows) != len(SEEDS):
                    raise RuntimeError(
                        f"{case.key} {variant.label} has {len(rows)} rows; "
                        f"expected {len(SEEDS)}"
                    )
                advance = successes > 0
                parallel_results.append(
                    {
                        "method": variant.label,
                        "completed_trials": len(rows),
                        "successes": successes,
                        "advance": advance,
                    }
                )
                if not advance:
                    active_parallel.remove(variant.label)
                    print(
                        f"PRUNE {scenario} {variant.label} after n={size}: "
                        f"0/{len(rows)} successes",
                        flush=True,
                    )
            decision = {
                "scenario": scenario,
                "team_size": size,
                "arc_completed_trials": len(arc_rows),
                "arc_successes": arc_successes,
                "arc_success_rate": arc_successes / len(SEEDS),
                "stop_after_this_size": stop,
                "stop_for_arc_at_or_below_half": stop_for_success_rate,
                "stop_for_scenario_cap": stop_for_cap,
                "parallel_methods": parallel_results,
            }
            decisions.append(decision)
            write_progress(
                progress_path,
                phase_name="p_arc_scaling",
                scaling_decisions=decisions,
                main_decisions=[],
            )
            if stop:
                print(
                    f"SCALING STOP {scenario} at n={size}: "
                    f"ARC solved {arc_successes}/{len(SEEDS)}; "
                    f"success_threshold={stop_for_success_rate}; "
                    f"hard_cap={stop_for_cap}",
                    flush=True,
                )
                break
            size *= 2
    return decisions


def run_main_comparison(
    *,
    output_root: Path,
    result_csv: Path,
    event_csv: Path,
    build_dir: Path,
    progress_path: Path,
    scaling_decisions: Sequence[dict[str, object]],
) -> list[dict[str, object]]:
    decisions: list[dict[str, object]] = []
    for scenario, sizes in MAIN_CASES.items():
        variants = main_variants(scenario)
        active = {variant.label for variant in variants}
        for size in sizes:
            case = benchmark_case(scenario, size)
            phase(
                f"Main 2D comparison: {case.key}; "
                f"active={', '.join(sorted(active))}"
            )
            for variant in variants:
                if variant.label not in active:
                    print(f"pruned: {case.key} {variant.label}", flush=True)
                    continue
                run_variant(
                    case=case,
                    variant=variant,
                    output_root=output_root,
                    result_csv=result_csv,
                    event_csv=event_csv,
                    build_dir=build_dir,
                )

            for variant in variants:
                if variant.label not in active:
                    continue
                rows = result_rows_for(
                    result_csv, case=case.key, method=variant.label
                )
                successes = success_count(
                    result_csv, case=case.key, method=variant.label
                )
                if len(rows) != len(SEEDS):
                    raise RuntimeError(
                        f"{case.key} {variant.label} has {len(rows)} rows; "
                        f"expected {len(SEEDS)}"
                    )
                advance = successes > 0
                decisions.append(
                    {
                        "scenario": scenario,
                        "team_size": size,
                        "method": variant.label,
                        "completed_trials": len(rows),
                        "successes": successes,
                        "advance": advance,
                    }
                )
                if not advance:
                    active.remove(variant.label)
                    print(
                        f"PRUNE {scenario} {variant.label} after n={size}: "
                        f"0/{len(rows)} successes",
                        flush=True,
                    )
            write_progress(
                progress_path,
                phase_name="main_2d_comparison",
                scaling_decisions=scaling_decisions,
                main_decisions=decisions,
            )
    return decisions


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--build-dir", type=Path, default=DEFAULT_BUILD_DIR)
    parser.add_argument("--max-cores", type=int, default=MAX_CORES)
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument(
        "--only",
        choices=("all", "scaling", "main"),
        default="all",
        help="Run the full campaign or one phase. Full runs always scale first.",
    )
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def print_dry_run(config: dict[str, object], build_dir: Path) -> None:
    print(json.dumps(config, indent=2))
    for scenario in SCENARIOS:
        case = benchmark_case(scenario, 4)
        for variant in scaling_variants(scenario):
            specs = build_trial_specs(
                cases=[case],
                variants=[variant],
                seeds=[0],
                task_indices=[0],
                time_limit=TIME_LIMIT_SECONDS,
                collision_backend=COLLISION_BACKEND,
                resolution=RESOLUTION,
                build_dir=build_dir,
                output_root=DEFAULT_OUTPUT_ROOT,
            )
            print("SCALING_COMMAND", " ".join(specs[0].command()))
    for scenario, sizes in MAIN_CASES.items():
        case = benchmark_case(scenario, sizes[0])
        for variant in main_variants(scenario):
            specs = build_trial_specs(
                cases=[case],
                variants=[variant],
                seeds=[0],
                task_indices=[0],
                time_limit=TIME_LIMIT_SECONDS,
                collision_backend=COLLISION_BACKEND,
                resolution=RESOLUTION,
                build_dir=build_dir,
                output_root=DEFAULT_OUTPUT_ROOT,
            )
            print("MAIN_COMMAND", " ".join(specs[0].command()))


def main() -> int:
    args = parse_args()
    if args.max_cores < MAX_CORES:
        raise RuntimeError(
            f"This campaign requires at least {MAX_CORES} available cores"
        )

    config = experiment_config()
    if args.dry_run:
        print_dry_run(config, args.build_dir)
        return 0

    output_root: Path = args.output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    write_json(output_root / "experiment_config.json", config)

    scaling_csv = output_root / "p_arc_scaling.csv"
    scaling_events = output_root / "p_arc_scaling_solution_events.csv"
    main_csv = output_root / "main_methods.csv"
    main_events = output_root / "main_solution_events.csv"
    progress_path = output_root / "progress.json"

    if not args.skip_build:
        phase("building the two 2D benchmark executables")
        subprocess.run(
            [
                "cmake",
                "--build",
                str(args.build_dir.parent),
                "--target",
                "mobile_robot_2d_crossing",
                "planar_manipulator_cross",
                "-j",
                str(args.max_cores),
            ],
            check=True,
        )

    scaling_decisions: list[dict[str, object]] = []
    if args.only in ("all", "scaling"):
        phase("phase 1: adaptive P-ARC worker scaling")
        scaling_decisions = run_scaling(
            output_root=output_root,
            result_csv=scaling_csv,
            event_csv=scaling_events,
            build_dir=args.build_dir,
            progress_path=progress_path,
        )

    main_decisions: list[dict[str, object]] = []
    if args.only in ("all", "main"):
        if not scaling_csv.is_file():
            if args.only == "all":
                raise RuntimeError("Scaling phase did not produce its results CSV")
        else:
            copy_scaling_overlap_to_main(
                scaling_results=scaling_csv,
                scaling_events=scaling_events,
                main_results=main_csv,
                main_events=main_events,
            )
        phase("phase 2: main eight-method 2D paper comparisons")
        main_decisions = run_main_comparison(
            output_root=output_root,
            result_csv=main_csv,
            event_csv=main_events,
            build_dir=args.build_dir,
            progress_path=progress_path,
            scaling_decisions=scaling_decisions,
        )

    phase("finalizing sorted CSVs and plots")
    if scaling_csv.is_file():
        finish_outputs(
            output_root=output_root / "p_arc_scaling",
            result_csv_path=scaling_csv,
            event_csv_path=scaling_events,
            result_rows=load_csv_rows(scaling_csv),
            event_rows=load_csv_rows(scaling_events),
            plot_kind="success",
        )
    if main_csv.is_file():
        finish_outputs(
            output_root=output_root / "main_methods",
            result_csv_path=main_csv,
            event_csv_path=main_events,
            result_rows=load_csv_rows(main_csv),
            event_rows=load_csv_rows(main_events),
            plot_kind="success",
        )
    write_progress(
        progress_path,
        phase_name="complete",
        scaling_decisions=scaling_decisions,
        main_decisions=main_decisions,
    )
    print(f"output_root: {output_root}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
