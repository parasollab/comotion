#!/usr/bin/env python3
"""Extend parc_final_8_20 Mobile Parallel P-ARC scaling to n=128/256."""

from __future__ import annotations

import json
import sys
from datetime import datetime, timezone
from pathlib import Path

sys.dont_write_bytecode = True

from benchmark_runner_common import DEFAULT_BUILD_DIR, finish_outputs, load_csv_rows
import run_parc_final_2d as campaign


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
ROOT = REPO_ROOT / "benchmarks" / "results" / "parc_final_8_20"
SIZES = (128, 256)
METHODS = ("ARC", "P-ARC-2", "P-ARC-4", "P-ARC-8", "P-ARC-16")
EXPECTED_HORIZON = 400


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def write_json(path: Path, value: object) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n")
    temporary.replace(path)


def verify() -> None:
    rows = load_csv_rows(ROOT / "p_arc_scaling.csv")
    for size in SIZES:
        case = f"mobile_parallel_n{size}"
        for method in METHODS:
            selected = [
                row
                for row in rows
                if row.get("case") == case and row.get("method") == method
            ]
            if len(selected) != 30:
                raise RuntimeError(
                    f"{case}/{method}: {len(selected)} rows, expected 30"
                )
            if method == "ARC":
                continue
            for row in selected:
                metrics_path = Path(str(row.get("metrics_json", "")))
                metrics = json.loads(metrics_path.read_text())
                horizon = metrics.get("planner_stats", {}).get(
                    "parallel_arc_conflict_find_horizon"
                )
                if horizon != EXPECTED_HORIZON:
                    raise RuntimeError(
                        f"{case}/{method}/seed={row.get('seed')}: horizon={horizon}"
                    )


def main() -> int:
    scaling_csv = ROOT / "p_arc_scaling.csv"
    scaling_events = ROOT / "p_arc_scaling_solution_events.csv"
    main_csv = ROOT / "main_methods.csv"
    main_events = ROOT / "main_solution_events.csv"
    progress = ROOT / "mobile_parallel_scaling_128_256_progress.json"
    write_json(progress, {"status": "running", "updated_utc": utc_now()})

    for size in SIZES:
        case = campaign.benchmark_case("mobile_parallel", size)
        print(f"\n[{utc_now()}] scaling {case.key}", flush=True)
        for variant in campaign.scaling_variants("mobile_parallel"):
            campaign.run_variant(
                case=case,
                variant=variant,
                output_root=ROOT,
                result_csv=scaling_csv,
                event_csv=scaling_events,
                build_dir=DEFAULT_BUILD_DIR,
            )

    campaign.copy_scaling_overlap_to_main(
        scaling_results=scaling_csv,
        scaling_events=scaling_events,
        main_results=main_csv,
        main_events=main_events,
    )
    finish_outputs(
        output_root=ROOT / "p_arc_scaling",
        result_csv_path=scaling_csv,
        event_csv_path=scaling_events,
        result_rows=load_csv_rows(scaling_csv),
        event_rows=load_csv_rows(scaling_events),
        plot_kind="success",
    )
    finish_outputs(
        output_root=ROOT / "main_methods",
        result_csv_path=main_csv,
        event_csv_path=main_events,
        result_rows=load_csv_rows(main_csv),
        event_rows=load_csv_rows(main_events),
        plot_kind="success",
    )
    verify()

    config_path = ROOT / "experiment_config.json"
    config = json.loads(config_path.read_text())
    config["mobile_parallel_scaling_extension"] = {
        "schema": "comotion.mobile_parallel_scaling_extension.v1",
        "completed_utc": utc_now(),
        "team_sizes": list(SIZES),
        "methods": list(METHODS),
        "parallel_arc_conflict_find_horizon": EXPECTED_HORIZON,
        "top_level_trial_jobs": 1,
    }
    write_json(config_path, config)
    write_json(progress, {"status": "complete", "updated_utc": utc_now()})
    print(f"\n[{utc_now()}] Mobile Parallel scaling extension complete", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
