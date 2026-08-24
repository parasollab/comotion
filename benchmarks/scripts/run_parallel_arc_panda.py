#!/usr/bin/env python3
"""Reproduce the final P-ARC Panda Cage experiment matrix."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True

from benchmark_runner_common import (
    CASE_CATALOG,
    DEFAULT_BUILD_DIR,
    DEFAULT_PARALLEL_ARC_CONFLICT_FIND_ASSIGNMENT,
    DEFAULT_RESULTS_DIR,
    EVENT_COLUMNS,
    RESULT_COLUMNS,
    PlannerVariant,
    build_trial_specs,
    effective_variant_extra_args,
    finish_outputs,
    load_csv_rows,
    run_trials,
    timestamp,
    truthy,
    write_manifest,
)


SEEDS = tuple(range(10))
TASK_INDICES = tuple(range(5))
TIME_LIMIT_SECONDS = 100.0
TIMEOUT_GRACE_SECONDS = 30.0
SIZES = (4, 8, 16)

PANDA_ARC_ARGS = (
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

PANDA_PARALLEL_ARGS = (
    "--parallel-arc-parallel-initial-plans",
    "--parallel-arc-initial-solution-or",
    "--parallel-arc-repair-duplicate-attempts",
    "--parallel-arc-strategy", "synchronous",
    "--parallel-arc-conflict-strategy", "greedy",
    "--parallel-arc-conflict-find-mode", "segment_parallel",
    "--parallel-arc-conflict-batch-mode", "optimistic",
    "--parallel-arc-conflict-find-horizon", "200",
)


def variants() -> list[PlannerVariant]:
    groups = [
        PlannerVariant("ARC", "arc", "arc", PANDA_ARC_ARGS),
        PlannerVariant(
            "P-ARC-16", "parallel_arc", "p_arc_16",
            ("--parallel-arc-worker-processes", "16",
             *PANDA_PARALLEL_ARGS, *PANDA_ARC_ARGS),
        ),
        PlannerVariant(
            "OR-ARC-16", "arc", "or_arc_16",
            ("--or-parallel-worker-processes", "16", *PANDA_ARC_ARGS),
        ),
    ]
    for outer, inner in ((2, 8), (4, 4), (8, 2)):
        groups.append(
            PlannerVariant(
                f"OR-P-ARC-{outer}x{inner}", "parallel_arc",
                f"or_p_arc_{outer}x{inner}",
                ("--or-parallel-worker-processes", str(outer),
                 "--parallel-arc-worker-processes", str(inner),
                 *PANDA_PARALLEL_ARGS, *PANDA_ARC_ARGS),
            )
        )
    groups.extend(
        [
            PlannerVariant(
                "EP-RRT-C-16", "cooperative_composite", "ep_rrt_c_16",
                ("--cooperative-rrt-worker-threads", "16"),
            ),
            PlannerVariant(
                "OR-RRT-C-16", "composite", "or_rrt_c_16",
                ("--or-parallel-worker-processes", "16"),
            ),
            PlannerVariant(
                "OR-EP-RRT-C-4x4", "cooperative_composite",
                "or_ep_rrt_c_4x4",
                ("--or-parallel-worker-processes", "4",
                 "--cooperative-rrt-worker-threads", "4"),
            ),
            PlannerVariant(
                "OR-PP-ST-RRT-16", "prioritized", "or_pp_st_rrt_16",
                ("--or-parallel-worker-processes", "16",
                 "--strrt-shuffle-priority-order",
                 "--strrt-rewiring", "off"),
            ),
        ]
    )
    return groups


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--build-dir", type=Path, default=DEFAULT_BUILD_DIR)
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the standard profile and representative commands only.",
    )
    return parser.parse_args()


def experiment_config(methods: list[PlannerVariant]) -> dict[str, object]:
    return {
        "schema": "comotion.parallel_arc_panda.v1",
        "paper": (
            "P-ARC: Exploiting Subproblem Independence for Parallel "
            "Multi-Robot Motion Planning"
        ),
        "paper_doi": "https://doi.org/10.48550/arXiv.2606.27625",
        "seeds": list(SEEDS),
        "task_indices": list(TASK_INDICES),
        "team_sizes": list(SIZES),
        "time_limit_seconds": TIME_LIMIT_SECONDS,
        "timeout_grace_seconds": TIMEOUT_GRACE_SECONDS,
        "collision_backend": "vamp",
        "resolution": 128,
        "top_level_trial_jobs": 1,
        "validation_instrumentation": False,
        "arc_profile": list(PANDA_ARC_ARGS),
        "parallel_arc_profile": list(PANDA_PARALLEL_ARGS),
        "method_commands": {
            method.label: list(effective_variant_extra_args(method))
            for method in methods
        },
        "parallel_arc_conflict_find_assignment": (
            DEFAULT_PARALLEL_ARC_CONFLICT_FIND_ASSIGNMENT
        ),
        "arc_repair_seed_schedule": (
            "unique per outer trial, logical repair batch/task, and attempt; "
            "independent of worker slot"
        ),
        "zero_success_pruning": True,
        "trial_process_group_cleanup_required": True,
    }


def main() -> int:
    args = parse_args()
    output_root = args.output_root or (
        DEFAULT_RESULTS_DIR / f"parallel_arc_panda_{timestamp()}"
    )
    result_csv = output_root / "results.csv"
    event_csv = output_root / "solution_events.csv"
    pruning_path = output_root / "pruning_decisions.json"
    methods = variants()
    cases = [CASE_CATALOG[f"panda_cage_n{size}"] for size in SIZES]
    config = experiment_config(methods)

    if args.dry_run:
        print(json.dumps(config, indent=2))
        for case in cases:
            for method in methods:
                spec = build_trial_specs(
                    cases=[case], variants=[method], seeds=[0],
                    task_indices=[0], time_limit=TIME_LIMIT_SECONDS,
                    collision_backend="vamp", resolution=128,
                    build_dir=args.build_dir, output_root=output_root,
                )[0]
                print("COMMAND", " ".join(spec.command()))
        return 0

    output_root.mkdir(parents=True, exist_ok=True)

    write_manifest(
        output_root,
        experiment_type="parallel_arc_panda_cage",
        command_line=sys.argv,
        cases=cases,
        variants=methods,
        trial_count=len(SIZES) * len(methods) * len(SEEDS) * len(TASK_INDICES),
    )
    (output_root / "experiment_config.json").write_text(
        json.dumps(config, indent=2) + "\n"
    )

    if not args.skip_build:
        subprocess.run(
            ["cmake", "--build", str(args.build_dir.parent),
             "--target", "panda_cage", "-j", "16"],
            check=True,
        )

    active = {method.label for method in methods}
    decisions: list[dict[str, object]] = []
    for size in SIZES:
        case = CASE_CATALOG[f"panda_cage_n{size}"]
        for method in methods:
            if method.label not in active:
                print(f"pruned: {case.key} {method.label}", flush=True)
                continue
            specs = build_trial_specs(
                cases=[case], variants=[method], seeds=SEEDS,
                task_indices=TASK_INDICES, time_limit=TIME_LIMIT_SECONDS,
                collision_backend="vamp", resolution=128,
                build_dir=args.build_dir, output_root=output_root,
            )
            print(
                f"=== {case.key}/{method.label}: jobs=1 trials={len(specs)} ===",
                flush=True,
            )
            run_trials(
                specs, jobs=1,
                timeout_seconds=TIME_LIMIT_SECONDS + TIMEOUT_GRACE_SECONDS,
                keep_metrics_json=True, result_csv_path=result_csv,
                event_csv_path=event_csv, skip_existing=True,
            )

        rows = load_csv_rows(result_csv)
        expected_rows = len(SEEDS) * len(TASK_INDICES)
        for method in methods:
            if method.label not in active:
                continue
            method_rows = [
                row for row in rows
                if row.get("case") == case.key and
                row.get("method") == method.label
            ]
            if len(method_rows) != expected_rows:
                raise RuntimeError(
                    f"{case.key} {method.label} has {len(method_rows)} "
                    f"result rows; expected {expected_rows}"
                )
            successes = sum(truthy(row.get("success")) for row in method_rows)
            decision = {
                "scenario": "panda_cage", "team_size": size,
                "method": method.label, "completed_trials": len(method_rows),
                "successes": successes, "advance": successes > 0,
            }
            decisions.append(decision)
            if successes == 0:
                active.remove(method.label)
                print(
                    f"PRUNE {method.label} after n={size}: "
                    f"0/{len(method_rows)} successes",
                    flush=True,
                )
        pruning_path.write_text(
            json.dumps(
                {
                    "schema": "comotion.zero_success_pruning.v1",
                    "rule": "Prune only after zero successes at a team size.",
                    "decisions": decisions,
                },
                indent=2,
            ) + "\n"
        )

    finish_outputs(
        output_root=output_root, result_csv_path=result_csv,
        event_csv_path=event_csv, result_rows=load_csv_rows(result_csv),
        event_rows=load_csv_rows(event_csv), plot_kind="success",
    )
    print(f"output_root: {output_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
