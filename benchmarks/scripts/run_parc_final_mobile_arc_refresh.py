#!/usr/bin/env python3
"""Refresh the mobile ARC-family rows in parc_final_8_20.

The scaling matrix is ARC and P-ARC-{2,4,8,16}.  The main-comparison ARC
family is ARC, P-ARC-16, OR-ARC-16, and OR-P-ARC-4x4.  Top-level trials are
always serial.  Use --reset-only once to remove stale mobile ARC-family rows
and metrics, then run without it; an interrupted run is resumable.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

sys.dont_write_bytecode = True

from benchmark_runner_common import (
    DEFAULT_BUILD_DIR,
    DEFAULT_RESULTS_DIR,
    EVENT_COLUMNS,
    RESULT_COLUMNS,
    effective_variant_extra_args,
    finish_outputs,
    load_csv_rows,
    truthy,
    write_csv,
)
from run_parc_final_2d import (
    SEEDS,
    arc_args,
    benchmark_case,
    copy_scaling_overlap_to_main,
    main_variants,
    run_variant,
    scaling_variants,
)


DEFAULT_OUTPUT_ROOT = DEFAULT_RESULTS_DIR / "parc_final_8_20"
SCALING_SIZES = {
    "mobile_parallel": (4, 8, 16, 32, 64),
    "mobile_circle": (4, 8, 16),
}
MAIN_SIZES = {
    "mobile_parallel": (16, 64, 256),
    "mobile_circle": (4, 8, 16),
}
SCALING_METHODS = ("ARC", "P-ARC-2", "P-ARC-4", "P-ARC-8", "P-ARC-16")
MAIN_METHODS = ("ARC", "P-ARC-16", "OR-ARC-16", "OR-P-ARC-4x4")
METRIC_SLUGS = {
    "arc",
    "p_arc_2",
    "p_arc_4",
    "p_arc_8",
    "p_arc_16",
    "or_arc_16",
    "or_p_arc_4x4",
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def is_mobile_arc_family(row: dict[str, str]) -> bool:
    case = row.get("case", "")
    method = row.get("method", "")
    mobile = case.startswith("mobile_parallel_n") or case.startswith("mobile_circle_n")
    arc_family = (
        method == "ARC"
        or method.startswith("P-ARC-")
        or method.startswith("OR-ARC-")
        or method.startswith("OR-P-ARC-")
    )
    return mobile and arc_family


def reset_existing(output_root: Path) -> dict[str, object]:
    removed_rows: dict[str, int] = {}
    for name, columns in (
        ("p_arc_scaling.csv", RESULT_COLUMNS),
        ("p_arc_scaling_solution_events.csv", EVENT_COLUMNS),
        ("main_methods.csv", RESULT_COLUMNS),
        ("main_solution_events.csv", EVENT_COLUMNS),
    ):
        path = output_root / name
        rows = load_csv_rows(path)
        retained = [row for row in rows if not is_mobile_arc_family(row)]
        removed_rows[name] = len(rows) - len(retained)
        write_csv(path, columns, retained)

    removed_metric_directories: list[str] = []
    metrics_root = output_root / "metrics"
    if metrics_root.is_dir():
        for case_dir in sorted(metrics_root.iterdir()):
            if not case_dir.is_dir() or not case_dir.name.startswith(
                ("mobile_parallel_n", "mobile_circle_n")
            ):
                continue
            for slug in sorted(METRIC_SLUGS):
                target = case_dir / slug
                if target.is_dir():
                    shutil.rmtree(target)
                    removed_metric_directories.append(str(target.relative_to(output_root)))

    record = {
        "schema": "comotion.parc_final_mobile_arc_reset.v1",
        "reset_utc": utc_now(),
        "output_root": str(output_root),
        "removed_rows": removed_rows,
        "removed_metric_directory_count": len(removed_metric_directories),
        "removed_metric_directories": removed_metric_directories,
    }
    path = output_root / "mobile_arc_refresh_reset_manifest.json"
    path.write_text(json.dumps(record, indent=2) + "\n")
    return record


def success_count(path: Path, case: str, method: str) -> int:
    return sum(
        truthy(row.get("success"))
        for row in load_csv_rows(path)
        if row.get("case") == case and row.get("method") == method
    )


def completed_count(path: Path, case: str, method: str) -> int:
    return sum(
        1
        for row in load_csv_rows(path)
        if row.get("case") == case and row.get("method") == method
    )


def write_progress(path: Path, phase: str, decisions: list[dict[str, object]]) -> None:
    path.write_text(
        json.dumps(
            {
                "schema": "comotion.parc_final_mobile_arc_refresh_progress.v1",
                "updated_utc": utc_now(),
                "phase": phase,
                "decisions": decisions,
            },
            indent=2,
        )
        + "\n"
    )


def run_refresh(output_root: Path, build_dir: Path, skip_build: bool) -> None:
    if not skip_build:
        subprocess.run(
            [
                "cmake",
                "--build",
                str(build_dir.parent),
                "--target",
                "mobile_robot_2d_crossing",
                "-j",
                "16",
            ],
            check=True,
        )

    scaling_csv = output_root / "p_arc_scaling.csv"
    scaling_events = output_root / "p_arc_scaling_solution_events.csv"
    main_csv = output_root / "main_methods.csv"
    main_events = output_root / "main_solution_events.csv"
    progress_path = output_root / "mobile_arc_refresh_progress.json"
    decisions: list[dict[str, object]] = []

    config = {
        "schema": "comotion.parc_final_mobile_arc_refresh.v1",
        "created_utc": utc_now(),
        "output_root": str(output_root),
        "seeds": list(SEEDS),
        "time_limit_seconds": 30.0,
        "resolution": 128,
        "collision_backend": "vamp",
        "top_level_trial_jobs": 1,
        "scaling_sizes": {key: list(value) for key, value in SCALING_SIZES.items()},
        "main_sizes": {key: list(value) for key, value in MAIN_SIZES.items()},
        "scaling_methods": list(SCALING_METHODS),
        "main_methods": list(MAIN_METHODS),
        "mobile_arc_args": list(arc_args("mobile_parallel")),
        "larger_size_pruning": "prune a method only after 0 successes in 30 trials",
    }
    (output_root / "mobile_arc_refresh_config.json").write_text(
        json.dumps(config, indent=2) + "\n"
    )
    root_config_path = output_root / "experiment_config.json"
    if root_config_path.is_file():
        root_config = json.loads(root_config_path.read_text())
        root_config.setdefault("arc_profiles", {})["mobile_parallel"] = list(
            arc_args("mobile_parallel")
        )
        root_config.setdefault("arc_profiles", {})["mobile_circle"] = list(
            arc_args("mobile_circle")
        )
        command_profiles = root_config.setdefault(
            "variant_commands_by_scenario", {}
        )
        for scenario in SCALING_SIZES:
            variants = {
                variant.label: list(effective_variant_extra_args(variant))
                for variant in (
                    *scaling_variants(scenario),
                    *main_variants(scenario),
                )
            }
            command_profiles[scenario] = variants
        root_config["mobile_arc_refresh"] = {
            "config": "mobile_arc_refresh_config.json",
            "reset_manifest": "mobile_arc_refresh_reset_manifest.json",
            "scaling_sizes": {
                key: list(value) for key, value in SCALING_SIZES.items()
            },
            "main_sizes": {key: list(value) for key, value in MAIN_SIZES.items()},
            "started_utc": utc_now(),
        }
        root_config_path.write_text(json.dumps(root_config, indent=2) + "\n")

    for scenario, sizes in SCALING_SIZES.items():
        variants = [
            variant
            for variant in scaling_variants(scenario)
            if variant.label in SCALING_METHODS
        ]
        active = {variant.label for variant in variants}
        for size in sizes:
            case = benchmark_case(scenario, size)
            for variant in variants:
                if variant.label not in active:
                    print(f"PRUNED {case.key} {variant.label}", flush=True)
                    continue
                run_variant(
                    case=case,
                    variant=variant,
                    output_root=output_root,
                    result_csv=scaling_csv,
                    event_csv=scaling_events,
                    build_dir=build_dir,
                )
                completed = completed_count(scaling_csv, case.key, variant.label)
                successes = success_count(scaling_csv, case.key, variant.label)
                decision = {
                    "phase": "scaling",
                    "scenario": scenario,
                    "team_size": size,
                    "method": variant.label,
                    "completed": completed,
                    "successes": successes,
                }
                decisions.append(decision)
                write_progress(progress_path, "scaling", decisions)
                if completed == len(SEEDS) and successes == 0:
                    active.remove(variant.label)

    copy_scaling_overlap_to_main(
        scaling_results=scaling_csv,
        scaling_events=scaling_events,
        main_results=main_csv,
        main_events=main_events,
    )

    for scenario, sizes in MAIN_SIZES.items():
        variants = [
            variant
            for variant in main_variants(scenario)
            if variant.label in MAIN_METHODS
        ]
        active = {variant.label for variant in variants}
        for size in sizes:
            case = benchmark_case(scenario, size)
            for variant in variants:
                if variant.label not in active:
                    print(f"PRUNED {case.key} {variant.label}", flush=True)
                    continue
                run_variant(
                    case=case,
                    variant=variant,
                    output_root=output_root,
                    result_csv=main_csv,
                    event_csv=main_events,
                    build_dir=build_dir,
                )
                completed = completed_count(main_csv, case.key, variant.label)
                successes = success_count(main_csv, case.key, variant.label)
                decision = {
                    "phase": "main",
                    "scenario": scenario,
                    "team_size": size,
                    "method": variant.label,
                    "completed": completed,
                    "successes": successes,
                }
                decisions.append(decision)
                write_progress(progress_path, "main", decisions)
                if completed == len(SEEDS) and successes == 0:
                    active.remove(variant.label)

    finish_outputs(
        output_root=output_root / "p_arc_scaling",
        result_csv_path=scaling_csv,
        event_csv_path=scaling_events,
        result_rows=load_csv_rows(scaling_csv),
        event_rows=load_csv_rows(scaling_events),
        plot_kind="success",
    )
    finish_outputs(
        output_root=output_root / "main_methods",
        result_csv_path=main_csv,
        event_csv_path=main_events,
        result_rows=load_csv_rows(main_csv),
        event_rows=load_csv_rows(main_events),
        plot_kind="success",
    )
    write_progress(progress_path, "complete", decisions)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--build-dir", type=Path, default=DEFAULT_BUILD_DIR)
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--reset-only", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_root = args.output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    if args.reset_only:
        print(json.dumps(reset_existing(output_root), indent=2), flush=True)
        return 0
    run_refresh(output_root, args.build_dir.resolve(), args.skip_build)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
