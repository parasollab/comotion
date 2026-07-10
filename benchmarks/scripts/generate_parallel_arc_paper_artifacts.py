#!/usr/bin/env python3
"""Generate paper-style plots and tables from a parallel ARC reproduction run."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path
from typing import Any, Iterable, Sequence

sys.dont_write_bytecode = True

from benchmark_runner_common import (
    float_or_none,
    import_pyplot,
    method_color,
    truthy,
)


MAIN_METHODS = (
    "ARC",
    "P-ARC-16",
    "OR-ARC-16",
    "OR-P-ARC-4x4",
    "C-RRT-C-16",
    "OR-RRT-C",
    "OR-C-RRT-C-4x4",
    "OR-PP-ST-RRT-16",
)

PARC_METHODS = ("P-ARC-2", "P-ARC-4", "P-ARC-8", "P-ARC-16")
ALT_CUMULATIVE_METHODS = ("ARC", *PARC_METHODS)

PANDA_TABLE_METHODS = (
    "OR-ARC-16",
    "P-ARC-16",
    "OR-P-ARC-4x4",
)

MAIN_SCENARIO_GROUPS = {
    "mobile_parallel": {
        "title": "Mobile Parallel",
        "cases": ("mobile_parallel_n16", "mobile_parallel_n64", "mobile_parallel_n256"),
    },
    "mobile_circle": {
        "title": "Mobile Circle",
        "cases": ("mobile_circle_n4", "mobile_circle_n8", "mobile_circle_n16"),
    },
    "planar_cross": {
        "title": "Planar Cross",
        "cases": ("planar_cross_n16", "planar_cross_n64", "planar_cross_n256"),
    },
    "panda_cage": {
        "title": "Panda Cage",
        "cases": ("panda_cage_n4", "panda_cage_n8", "panda_cage_n16"),
    },
}

ALT_SCENARIO_GROUPS = {
    "mobile_parallel": {
        "title": "Mobile Parallel",
        "cases": (
            "mobile_parallel_n4",
            "mobile_parallel_n8",
            "mobile_parallel_n16",
            "mobile_parallel_n32",
            "mobile_parallel_n64",
            "mobile_parallel_n128",
            "mobile_parallel_n256",
        ),
    },
    "mobile_circle": {
        "title": "Mobile Circle",
        "cases": (
            "mobile_circle_n4",
            "mobile_circle_n8",
            "mobile_circle_n16",
            "mobile_circle_n32",
        ),
    },
    "planar_cross": {
        "title": "Planar Cross",
        "cases": (
            "planar_cross_n4",
            "planar_cross_n8",
            "planar_cross_n16",
            "planar_cross_n32",
            "planar_cross_n64",
            "planar_cross_n128",
            "planar_cross_n256",
        ),
    },
    "panda_cage": {
        "title": "Panda Cage",
        "cases": ("panda_cage_n4", "panda_cage_n8", "panda_cage_n16"),
    },
}

CASE_TITLES = {
    "mobile_parallel_n4": "Mobile Parallel - 4",
    "mobile_parallel_n8": "Mobile Parallel - 8",
    "mobile_parallel_n16": "Mobile Parallel - 16",
    "mobile_parallel_n32": "Mobile Parallel - 32",
    "mobile_parallel_n64": "Mobile Parallel - 64",
    "mobile_parallel_n128": "Mobile Parallel - 128",
    "mobile_parallel_n256": "Mobile Parallel - 256",
    "mobile_circle_n4": "Mobile Circle - 4",
    "mobile_circle_n8": "Mobile Circle - 8",
    "mobile_circle_n16": "Mobile Circle - 16",
    "mobile_circle_n32": "Mobile Circle - 32",
    "planar_cross_n4": "Planar Cross - 4",
    "planar_cross_n8": "Planar Cross - 8",
    "planar_cross_n16": "Planar Cross - 16",
    "planar_cross_n32": "Planar Cross - 32",
    "planar_cross_n64": "Planar Cross - 64",
    "planar_cross_n128": "Planar Cross - 128",
    "planar_cross_n256": "Planar Cross - 256",
    "panda_cage_n4": "Panda Cage - 4",
    "panda_cage_n8": "Panda Cage - 8",
    "panda_cage_n16": "Panda Cage - 16",
}


def robot_count(case_key: str) -> int:
    return int(case_key.rsplit("_n", 1)[1])


def case_sort_key(case_key: str) -> tuple[str, int]:
    return (case_key.rsplit("_n", 1)[0], robot_count(case_key))


def load_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return [dict(row) for row in csv.DictReader(handle)]


def write_csv(path: Path, rows: Sequence[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    columns: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for key in row:
            if key not in seen:
                seen.add(key)
                columns.append(key)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            writer.writerow({column: row.get(column, "") for column in columns})


def fmt(value: Any) -> str:
    if value is None or value == "":
        return ""
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        if abs(value) >= 100:
            return f"{value:.1f}"
        if abs(value) >= 10:
            return f"{value:.2f}"
        return f"{value:.3f}"
    return str(value)


def write_markdown_table(path: Path, rows: Sequence[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("")
        return
    columns = list(rows[0].keys())
    lines = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join("---" for _ in columns) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(fmt(row.get(column, "")) for column in columns) + " |")
    path.write_text("\n".join(lines) + "\n")


def write_latex_table(path: Path, rows: Sequence[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("")
        return
    columns = list(rows[0].keys())
    align = "l" + "r" * (len(columns) - 1)
    lines = [f"\\begin{{tabular}}{{{align}}}", "\\toprule"]
    lines.append(" & ".join(columns) + r" \\")
    lines.append("\\midrule")
    for row in rows:
        lines.append(" & ".join(fmt(row.get(column, "")) for column in columns) + r" \\")
    lines.extend(["\\bottomrule", "\\end{tabular}", ""])
    path.write_text("\n".join(lines))


def rows_for_case(rows: Sequence[dict[str, str]], case_key: str) -> list[dict[str, str]]:
    return [row for row in rows if row.get("case") == case_key]


def rows_for_method(rows: Sequence[dict[str, str]], method: str) -> list[dict[str, str]]:
    return [row for row in rows if row.get("method") == method]


def successful_first_solution_time(row: dict[str, str]) -> float | None:
    if not truthy(row.get("success")):
        return None
    return float_or_none(row.get("first_solution_time_seconds"))


def numeric_values(rows: Iterable[dict[str, str]], column: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = float_or_none(row.get(column))
        if value is not None:
            values.append(value)
    return values


def successful_values(rows: Iterable[dict[str, str]], column: str) -> list[float]:
    return [
        value
        for row in rows
        if truthy(row.get("success"))
        for value in [float_or_none(row.get(column))]
        if value is not None
    ]


def median_or_none(values: Sequence[float]) -> float | None:
    return statistics.median(values) if values else None


def log_axis_minimum(x_max: float, solve_times: Sequence[float]) -> float:
    positive_times = [value for value in solve_times if value > 0.0]
    if positive_times:
        return max(1e-3, min(positive_times) * 0.5)
    return max(1e-3, x_max * 1e-3)


def write_cumulative_success_plots(
    rows: Sequence[dict[str, str]],
    output_dir: Path,
    *,
    methods: Sequence[str],
    scenario_groups: dict[str, dict[str, Any]],
    plot_subdir: str = "cumulative_success",
) -> list[Path]:
    plt = import_pyplot(output_dir)
    if plt is None:
        return []

    plot_dir = output_dir / plot_subdir
    plot_dir.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []
    colors = method_color(methods)
    for group in scenario_groups.values():
        for case_key in group["cases"]:
            case_rows = rows_for_case(rows, case_key)
            if not case_rows:
                continue
            x_max = max(float(row["time_limit_seconds"]) for row in case_rows)
            case_solve_times = [
                value
                for row in case_rows
                for value in [successful_first_solution_time(row)]
                if value is not None and value > 0.0 and value <= x_max
            ]
            x_min = log_axis_minimum(x_max, case_solve_times)
            fig, ax = plt.subplots(figsize=(7.5, 4.8), dpi=180)
            plotted_any = False
            for method in methods:
                method_rows = rows_for_method(case_rows, method)
                if not method_rows:
                    continue
                plotted_any = True
                solve_times = sorted(
                    value
                    for row in method_rows
                    for value in [successful_first_solution_time(row)]
                    if value is not None and value <= x_max
                )
                n_trials = len(method_rows)
                x_values = [x_min]
                y_values = [0.0]
                solved = 0
                for solve_time in solve_times:
                    if solve_time <= 0.0:
                        continue
                    x_values.extend([solve_time, solve_time])
                    y_values.extend(
                        [100.0 * solved / n_trials, 100.0 * (solved + 1) / n_trials]
                    )
                    solved += 1
                x_values.append(x_max)
                y_values.append(100.0 * solved / n_trials)
                ax.step(
                    x_values,
                    y_values,
                    where="post",
                    linewidth=2.0,
                    color=colors[method],
                    label=method,
                )
            if not plotted_any:
                plt.close(fig)
                continue
            ax.set_title(CASE_TITLES.get(case_key, case_key))
            ax.set_xlabel("Runtime (s)")
            ax.set_ylabel("Cumulative success (%)")
            ax.set_xscale("log")
            ax.set_xlim(x_min, x_max)
            ax.set_ylim(0.0, 100.0)
            ax.grid(True, alpha=0.3)
            ax.legend(loc="lower right", fontsize=7)
            fig.tight_layout()
            for suffix in ("png", "svg", "pdf"):
                path = plot_dir / f"{case_key}_cumulative_success.{suffix}"
                fig.savefig(path)
                written.append(path)
            plt.close(fig)
    return written


def speedup_records(rows: Sequence[dict[str, str]]) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for group_key, group in ALT_SCENARIO_GROUPS.items():
        for case_key in group["cases"]:
            case_rows = rows_for_case(rows, case_key)
            arc_rows = rows_for_method(case_rows, "ARC")
            arc_median_runtime = median_or_none(numeric_values(arc_rows, "planning_time_seconds"))
            arc_median_first_solution = median_or_none(
                successful_values(arc_rows, "first_solution_time_seconds")
            )
            arc_successes = sum(1 for row in arc_rows if truthy(row.get("success")))
            for method in PARC_METHODS:
                method_rows = rows_for_method(case_rows, method)
                method_median_runtime = median_or_none(
                    numeric_values(method_rows, "planning_time_seconds")
                )
                method_median_first_solution = median_or_none(
                    successful_values(method_rows, "first_solution_time_seconds")
                )
                method_successes = sum(1 for row in method_rows if truthy(row.get("success")))
                speedup = (
                    arc_median_first_solution / method_median_first_solution
                    if arc_median_first_solution is not None
                    and method_median_first_solution is not None
                    and method_median_first_solution > 0.0
                    else None
                )
                all_trial_runtime_ratio = (
                    arc_median_runtime / method_median_runtime
                    if arc_median_runtime is not None
                    and method_median_runtime is not None
                    and method_median_runtime > 0.0
                    else None
                )
                records.append(
                    {
                        "scenario": group_key,
                        "case": case_key,
                        "robots": robot_count(case_key),
                        "method": method,
                        "cores": int(method.rsplit("-", 1)[1]),
                        "arc_median_runtime_seconds": arc_median_runtime,
                        "method_median_runtime_seconds": method_median_runtime,
                        "arc_median_first_solution_seconds": arc_median_first_solution,
                        "method_median_first_solution_seconds": method_median_first_solution,
                        "speedup_over_arc": speedup,
                        "all_trial_median_runtime_ratio": all_trial_runtime_ratio,
                        "arc_successes": arc_successes,
                        "method_successes": method_successes,
                        "trials": len(method_rows),
                    }
                )
    return records


def write_speedup_plots(
    records: Sequence[dict[str, Any]], output_dir: Path
) -> list[Path]:
    plt = import_pyplot(output_dir)
    if plt is None:
        return []

    plot_dir = output_dir / "speedup"
    plot_dir.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []
    colors = method_color(PARC_METHODS)
    for group_key, group in ALT_SCENARIO_GROUPS.items():
        fig, ax = plt.subplots(figsize=(6.6, 4.3), dpi=180)
        for method in PARC_METHODS:
            method_records = sorted(
                [
                    row
                    for row in records
                    if row["scenario"] == group_key and row["method"] == method
                ],
                key=lambda row: row["robots"],
            )
            xs = [row["robots"] for row in method_records]
            ys = [row["speedup_over_arc"] for row in method_records]
            ax.plot(
                xs,
                ys,
                marker="o",
                linewidth=2.0,
                color=colors[method],
                label=f"{row_label(method)} cores",
            )
        ax.axhline(1.0, color="#444444", linewidth=1.0, linestyle="--", alpha=0.7)
        robot_counts = [robot_count(case_key) for case_key in group["cases"]]
        ax.set_xticks(robot_counts)
        ax.set_title(f"{group['title']} Speedup")
        ax.set_xlabel("Number of robots")
        ax.set_ylabel("Median first-solution speedup over ARC")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        for suffix in ("png", "svg", "pdf"):
            path = plot_dir / f"{group_key}_parc_speedup.{suffix}"
            fig.savefig(path)
            written.append(path)
        plt.close(fig)
    return written


def row_label(method: str) -> str:
    return method.rsplit("-", 1)[1]


def summary_records(rows: Sequence[dict[str, str]]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for case_key in [
        case for group in MAIN_SCENARIO_GROUPS.values() for case in group["cases"]
    ]:
        case_rows = rows_for_case(rows, case_key)
        for method in MAIN_METHODS:
            method_rows = rows_for_method(case_rows, method)
            if not method_rows:
                continue
            successes = sum(1 for row in method_rows if truthy(row.get("success")))
            out.append(
                {
                    "case": case_key,
                    "robots": robot_count(case_key),
                    "method": method,
                    "trials": len(method_rows),
                    "successes": successes,
                    "success_rate_percent": 100.0 * successes / len(method_rows),
                    "median_runtime_seconds": median_or_none(
                        numeric_values(method_rows, "planning_time_seconds")
                    ),
                    "median_first_solution_seconds": median_or_none(
                        successful_values(method_rows, "first_solution_time_seconds")
                    ),
                    "median_makespan_timesteps": median_or_none(
                        successful_values(method_rows, "makespan_timesteps")
                    ),
                }
            )
    return out


def panda_speedup_table(rows: Sequence[dict[str, str]]) -> list[dict[str, Any]]:
    cases = MAIN_SCENARIO_GROUPS["panda_cage"]["cases"]
    table_rows: list[dict[str, Any]] = [
        {
            "method": "ARC median time",
            **{
                CASE_TITLES[case_key]: median_or_none(
                    numeric_values(
                        rows_for_method(rows_for_case(rows, case_key), "ARC"),
                        "planning_time_seconds",
                    )
                )
                for case_key in cases
            },
        }
    ]
    for method in PANDA_TABLE_METHODS:
        table_rows.append(
            {
                "method": f"{method} speedup",
                **{
                    CASE_TITLES[case_key]: speedup_value(rows, case_key, method)
                    for case_key in cases
                },
            }
        )
    return table_rows


def speedup_value(rows: Sequence[dict[str, str]], case_key: str, method: str) -> float | None:
    case_rows = rows_for_case(rows, case_key)
    arc = median_or_none(numeric_values(rows_for_method(case_rows, "ARC"), "planning_time_seconds"))
    method_value = median_or_none(
        numeric_values(rows_for_method(case_rows, method), "planning_time_seconds")
    )
    if arc is None or method_value is None or method_value <= 0.0:
        return None
    return arc / method_value


def panda_cost_table(rows: Sequence[dict[str, str]]) -> list[dict[str, Any]]:
    cases = MAIN_SCENARIO_GROUPS["panda_cage"]["cases"]
    table_rows: list[dict[str, Any]] = [
        {
            "method": "ARC median makespan",
            **{
                CASE_TITLES[case_key]: median_or_none(
                    successful_values(
                        rows_for_method(rows_for_case(rows, case_key), "ARC"),
                        "makespan_timesteps",
                    )
                )
                for case_key in cases
            },
        }
    ]
    for method in PANDA_TABLE_METHODS:
        table_rows.append(
            {
                "method": f"{method} relative cost",
                **{
                    CASE_TITLES[case_key]: relative_cost_value(rows, case_key, method)
                    for case_key in cases
                },
            }
        )
    return table_rows


def relative_cost_value(
    rows: Sequence[dict[str, str]], case_key: str, method: str
) -> float | None:
    case_rows = rows_for_case(rows, case_key)
    arc = median_or_none(
        successful_values(rows_for_method(case_rows, "ARC"), "makespan_timesteps")
    )
    method_value = median_or_none(
        successful_values(rows_for_method(case_rows, method), "makespan_timesteps")
    )
    if arc is None or method_value is None or arc <= 0.0:
        return None
    return method_value / arc


def write_table_family(output_dir: Path, stem: str, rows: Sequence[dict[str, Any]]) -> None:
    table_dir = output_dir / "tables"
    write_csv(table_dir / f"{stem}.csv", rows)
    write_markdown_table(table_dir / f"{stem}.md", rows)
    write_latex_table(table_dir / f"{stem}.tex", rows)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate paper-style plots and tables from reproduction CSVs."
    )
    parser.add_argument(
        "--results-root",
        type=Path,
        default=Path("benchmarks/results/new-parc-results"),
    )
    parser.add_argument("--output-dir", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    results_root = args.results_root
    output_dir = args.output_dir or results_root / "paper_artifacts"
    output_dir.mkdir(parents=True, exist_ok=True)

    main_rows = load_csv(results_root / "main_methods.csv")
    alt_rows = load_csv(results_root / "alternative_cores.csv")

    alt_success_plots = write_cumulative_success_plots(
        alt_rows,
        output_dir,
        methods=ALT_CUMULATIVE_METHODS,
        scenario_groups=ALT_SCENARIO_GROUPS,
        plot_subdir="cumulative_success",
    )
    main_success_plots = write_cumulative_success_plots(
        main_rows,
        output_dir,
        methods=MAIN_METHODS,
        scenario_groups=MAIN_SCENARIO_GROUPS,
        plot_subdir="cumulative_success_main_methods",
    )
    speedups = speedup_records(alt_rows)
    speedup_plots = write_speedup_plots(speedups, output_dir)

    write_table_family(output_dir, "main_method_summary", summary_records(main_rows))
    write_table_family(output_dir, "parc_speedups_over_arc", speedups)
    write_table_family(output_dir, "panda_speedups", panda_speedup_table(main_rows))
    write_table_family(output_dir, "panda_relative_costs", panda_cost_table(main_rows))

    print(f"output_dir: {output_dir}")
    print(f"cumulative_success_plots_alt_cases: {len(alt_success_plots)}")
    print(f"cumulative_success_plots_main_methods: {len(main_success_plots)}")
    print(f"speedup_plots: {len(speedup_plots)}")
    print(f"tables_dir: {output_dir / 'tables'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
