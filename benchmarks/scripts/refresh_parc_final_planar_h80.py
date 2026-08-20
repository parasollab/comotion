#!/usr/bin/env python3
"""Replace only planar-cross ARC-family results with horizon-80 trials."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path

sys.dont_write_bytecode = True

from benchmark_runner_common import (
    DEFAULT_BUILD_DIR,
    EVENT_COLUMNS,
    RESULT_COLUMNS,
    finish_outputs,
    load_csv_rows,
    write_csv,
)
import run_parc_final_2d as campaign


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_ROOT = REPO_ROOT / "benchmarks" / "results" / "parc_final_8_20"
DEFAULT_IMPORT_ROOT = (
    REPO_ROOT
    / "benchmarks"
    / "results"
    / "planar_cross_horizon80_seeds0-3_n64_n128_20260820"
)

PLANAR_SIZES = (4, 8, 16, 32, 64, 128, 256)
MAIN_SIZES = (16, 64, 256)
SEEDS = tuple(range(30))
HORIZON = 80
ARC_FAMILY_METHODS = {
    "ARC",
    "P-ARC-2",
    "P-ARC-4",
    "P-ARC-8",
    "P-ARC-16",
    "OR-ARC-16",
    "OR-P-ARC-4x4",
}
ARC_FAMILY_SLUGS = {
    "arc",
    "p_arc_2",
    "p_arc_4",
    "p_arc_8",
    "p_arc_16",
    "or_arc_16",
    "or_p_arc_4x4",
}
SCALING_METHODS = {"ARC", "P-ARC-2", "P-ARC-4", "P-ARC-8", "P-ARC-16"}
MAIN_REFRESH_METHODS = {"OR-ARC-16", "OR-P-ARC-4x4"}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n")
    temporary.replace(path)


def is_planar_arc_row(row: dict[str, object]) -> bool:
    return str(row.get("case", "")).startswith("planar_cross_n") and str(
        row.get("method", "")
    ) in ARC_FAMILY_METHODS


def configure_horizon() -> None:
    args = list(campaign.PARALLEL_ARC_ARGS)
    index = args.index("--parallel-arc-conflict-find-horizon")
    args[index + 1] = str(HORIZON)
    campaign.PARALLEL_ARC_ARGS = tuple(args)


def reset_arc_family(root: Path, marker: Path) -> None:
    if marker.is_file():
        print(f"reset already complete: {marker}", flush=True)
        return

    if root.name != "parc_final_8_20":
        raise RuntimeError(f"refusing reset for unexpected directory: {root}")

    csv_paths = (
        root / "p_arc_scaling.csv",
        root / "p_arc_scaling_solution_events.csv",
        root / "main_methods.csv",
        root / "main_solution_events.csv",
    )
    reset_counts: dict[str, dict[str, int]] = {}
    for path in csv_paths:
        rows = load_csv_rows(path)
        retained = [row for row in rows if not is_planar_arc_row(row)]
        columns = EVENT_COLUMNS if "events" in path.name else RESULT_COLUMNS
        write_csv(path, columns, retained)
        reset_counts[path.name] = {
            "before": len(rows),
            "removed": len(rows) - len(retained),
            "retained": len(retained),
        }

    removed_metric_dirs: list[str] = []
    for size in PLANAR_SIZES:
        case_dir = root / "metrics" / f"planar_cross_n{size}"
        for slug in sorted(ARC_FAMILY_SLUGS):
            target = case_dir / slug
            if target.is_dir():
                shutil.rmtree(target)
                removed_metric_dirs.append(str(target.relative_to(root)))

    write_json(
        marker,
        {
            "schema": "comotion.planar_arc_h80_reset.v1",
            "completed_utc": utc_now(),
            "target": str(root),
            "arc_family_methods_removed": sorted(ARC_FAMILY_METHODS),
            "csv_counts": reset_counts,
            "removed_metric_directories": removed_metric_dirs,
            "baseline_rows_and_metrics_preserved": True,
        },
    )
    print(f"ARC-family planar reset complete: {marker}", flush=True)


def result_key(row: dict[str, object]) -> tuple[str, str, str, str]:
    return (
        str(row.get("case", "")),
        str(row.get("task_index", "")),
        str(row.get("seed", "")),
        str(row.get("method", "")),
    )


def event_key(row: dict[str, object]) -> tuple[str, str, str, str, str, str]:
    return (
        *result_key(row),
        str(row.get("elapsed_seconds", "")),
        str(row.get("makespan_timesteps", "")),
    )


def copy_imported_trials(root: Path, import_root: Path) -> None:
    scaling_csv = root / "p_arc_scaling.csv"
    scaling_events = root / "p_arc_scaling_solution_events.csv"
    rows = load_csv_rows(scaling_csv)
    events = load_csv_rows(scaling_events)
    row_keys = {result_key(row) for row in rows}
    event_keys = {event_key(row) for row in events}
    copied = 0

    for size in (64, 128):
        source_dir = import_root / f"n{size}"
        for source_row in load_csv_rows(source_dir / "results.csv"):
            if str(source_row.get("method", "")) not in SCALING_METHODS:
                continue
            row = dict(source_row)
            source_metrics = Path(str(row["metrics_json"]))
            relative_metrics = source_metrics.relative_to(source_dir)
            destination_metrics = root / relative_metrics
            destination_metrics.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source_metrics, destination_metrics)
            row["metrics_json"] = str(destination_metrics.resolve())
            key = result_key(row)
            if key not in row_keys:
                rows.append(row)
                row_keys.add(key)
                copied += 1

        for source_event in load_csv_rows(source_dir / "solution_events.csv"):
            if str(source_event.get("method", "")) not in SCALING_METHODS:
                continue
            event = dict(source_event)
            key = event_key(event)
            if key not in event_keys:
                events.append(event)
                event_keys.add(key)

    write_csv(scaling_csv, RESULT_COLUMNS, rows)
    write_csv(scaling_events, EVENT_COLUMNS, events)
    print(f"copied {copied} horizon-80 result rows for n=64/n=128", flush=True)


def run_scaling(root: Path, build_dir: Path) -> None:
    result_csv = root / "p_arc_scaling.csv"
    event_csv = root / "p_arc_scaling_solution_events.csv"
    for size in PLANAR_SIZES:
        case = campaign.benchmark_case("planar_cross", size)
        print(f"\n[{utc_now()}] scaling {case.key}", flush=True)
        for variant in campaign.scaling_variants("planar_cross"):
            campaign.run_variant(
                case=case,
                variant=variant,
                output_root=root,
                result_csv=result_csv,
                event_csv=event_csv,
                build_dir=build_dir,
            )


def run_main_arc_variants(root: Path, build_dir: Path) -> None:
    scaling_csv = root / "p_arc_scaling.csv"
    scaling_events = root / "p_arc_scaling_solution_events.csv"
    main_csv = root / "main_methods.csv"
    main_events = root / "main_solution_events.csv"
    campaign.copy_scaling_overlap_to_main(
        scaling_results=scaling_csv,
        scaling_events=scaling_events,
        main_results=main_csv,
        main_events=main_events,
    )

    variants = [
        variant
        for variant in campaign.main_variants("planar_cross")
        if variant.label in MAIN_REFRESH_METHODS
    ]
    for size in MAIN_SIZES:
        case = campaign.benchmark_case("planar_cross", size)
        print(f"\n[{utc_now()}] main ARC variants {case.key}", flush=True)
        for variant in variants:
            campaign.run_variant(
                case=case,
                variant=variant,
                output_root=root,
                result_csv=main_csv,
                event_csv=main_events,
                build_dir=build_dir,
            )


def finalize(root: Path) -> None:
    scaling_csv = root / "p_arc_scaling.csv"
    scaling_events = root / "p_arc_scaling_solution_events.csv"
    main_csv = root / "main_methods.csv"
    main_events = root / "main_solution_events.csv"
    finish_outputs(
        output_root=root / "p_arc_scaling",
        result_csv_path=scaling_csv,
        event_csv_path=scaling_events,
        result_rows=load_csv_rows(scaling_csv),
        event_rows=load_csv_rows(scaling_events),
        plot_kind="success",
    )
    finish_outputs(
        output_root=root / "main_methods",
        result_csv_path=main_csv,
        event_csv_path=main_events,
        result_rows=load_csv_rows(main_csv),
        event_rows=load_csv_rows(main_events),
        plot_kind="success",
    )

    config_path = root / "experiment_config.json"
    config = json.loads(config_path.read_text())
    config["planar_cross_arc_refresh"] = {
        "schema": "comotion.planar_arc_h80_refresh.v1",
        "completed_utc": utc_now(),
        "seeds": list(SEEDS),
        "team_sizes": list(PLANAR_SIZES),
        "scaling_methods": sorted(SCALING_METHODS),
        "main_team_sizes": list(MAIN_SIZES),
        "main_methods": ["ARC", "P-ARC-16", *sorted(MAIN_REFRESH_METHODS)],
        "parallel_arc_conflict_find_horizon": HORIZON,
        "top_level_trial_jobs": 1,
    }
    write_json(config_path, config)
    write_json(
        root / "planar_arc_horizon80_refresh_progress.json",
        {"status": "complete", "updated_utc": utc_now()},
    )


def verify(root: Path) -> None:
    scaling = load_csv_rows(root / "p_arc_scaling.csv")
    main = load_csv_rows(root / "main_methods.csv")
    expected_scaling = {
        (f"planar_cross_n{size}", method)
        for size in PLANAR_SIZES
        for method in SCALING_METHODS
    }
    expected_main = {
        (f"planar_cross_n{size}", method)
        for size in MAIN_SIZES
        for method in ("ARC", "P-ARC-16", *MAIN_REFRESH_METHODS)
    }
    for rows, expected, label in (
        (scaling, expected_scaling, "scaling"),
        (main, expected_main, "main"),
    ):
        for case, method in sorted(expected):
            selected = [
                row
                for row in rows
                if row.get("case") == case and row.get("method") == method
            ]
            if len(selected) != len(SEEDS):
                raise RuntimeError(
                    f"{label}: {case}/{method} has {len(selected)} rows, "
                    f"expected {len(SEEDS)}"
                )
            for row in selected:
                if method.startswith("P-ARC") or method == "OR-P-ARC-4x4":
                    metrics = json.loads(Path(str(row["metrics_json"])).read_text())
                    actual = metrics.get("planner_stats", {}).get(
                        "parallel_arc_conflict_find_horizon"
                    )
                    if actual != HORIZON:
                        raise RuntimeError(
                            f"{case}/{method}/seed={row.get('seed')} horizon={actual}"
                        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--import-root", type=Path, default=DEFAULT_IMPORT_ROOT)
    parser.add_argument("--build-dir", type=Path, default=DEFAULT_BUILD_DIR)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = args.root.resolve()
    import_root = args.import_root.resolve()
    marker = root / "planar_arc_horizon80_reset_manifest.json"
    configure_horizon()
    reset_arc_family(root, marker)
    copy_imported_trials(root, import_root)
    write_json(
        root / "planar_arc_horizon80_refresh_progress.json",
        {"status": "running", "updated_utc": utc_now()},
    )
    run_scaling(root, args.build_dir.resolve())
    run_main_arc_variants(root, args.build_dir.resolve())
    finalize(root)
    verify(root)
    print(f"\n[{utc_now()}] planar ARC horizon-80 refresh complete", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
