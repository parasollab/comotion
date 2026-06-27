from __future__ import annotations

import csv
import json
import math
import os
import statistics
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if (
    len(SCRIPT_DIR.parents) >= 4
    and SCRIPT_DIR.parents[1].name == "comotion"
    and SCRIPT_DIR.parents[2].name == "share"
):
    INSTALL_PREFIX = SCRIPT_DIR.parents[3]
    RUNTIME_CWD = INSTALL_PREFIX / "share" / "comotion"
    DEFAULT_BUILD_DIR = INSTALL_PREFIX / "bin"
    DEFAULT_RESULTS_DIR = Path.cwd() / "comotion-results"
else:
    REPO_ROOT = SCRIPT_DIR.parents[1]
    RUNTIME_CWD = REPO_ROOT
    DEFAULT_BUILD_DIR = REPO_ROOT / "build" / "apps"
    DEFAULT_RESULTS_DIR = REPO_ROOT / "benchmarks" / "results"
_MPL_CACHE_DIR = None


@dataclass(frozen=True)
class BenchmarkCase:
    key: str
    title: str
    executable: str
    base_args: tuple[str, ...]
    task_based: bool = False


@dataclass(frozen=True)
class PlannerVariant:
    label: str
    algorithm: str
    slug: str
    extra_args: tuple[str, ...] = ()


@dataclass(frozen=True)
class TrialSpec:
    case: BenchmarkCase
    variant: PlannerVariant
    seed: int
    task_index: int | None
    time_limit: float
    collision_backend: str
    resolution: int
    build_dir: Path
    output_root: Path

    @property
    def task_label(self) -> str:
        return "task_none" if self.task_index is None else f"task_{self.task_index:02d}"

    def command(self, metrics_path: Path | str = "<metrics-json>") -> list[str]:
        executable = self.build_dir / self.case.executable
        command = [
            str(executable),
            *self.case.base_args,
            "--algorithm",
            self.variant.algorithm,
            "--collision-backend",
            self.collision_backend,
            "--time-limit",
            format_float(self.time_limit),
            "--seed",
            str(self.seed),
            "--resolution",
            str(self.resolution),
            "--metrics-json",
            str(metrics_path),
        ]
        if self.case.task_based:
            if self.task_index is None:
                raise RuntimeError(f"{self.case.key} requires a task index")
            command.extend(["--task-index", str(self.task_index)])
        command.extend(self.variant.extra_args)
        return command


CASE_CATALOG: dict[str, BenchmarkCase] = {
    "mobile_parallel_n4": BenchmarkCase(
        key="mobile_parallel_n4",
        title="Mobile 2D parallel, 4 robots",
        executable="mobile_robot_2d_crossing",
        base_args=("--scenario", "parallel", "--num-robots", "4"),
    ),
    "mobile_parallel_n8": BenchmarkCase(
        key="mobile_parallel_n8",
        title="Mobile 2D parallel, 8 robots",
        executable="mobile_robot_2d_crossing",
        base_args=("--scenario", "parallel", "--num-robots", "8"),
    ),
    "mobile_parallel_n16": BenchmarkCase(
        key="mobile_parallel_n16",
        title="Mobile 2D parallel, 16 robots",
        executable="mobile_robot_2d_crossing",
        base_args=("--scenario", "parallel", "--num-robots", "16"),
    ),
    "mobile_circle_n4": BenchmarkCase(
        key="mobile_circle_n4",
        title="Mobile 2D circle, 4 robots",
        executable="mobile_robot_2d_crossing",
        base_args=("--scenario", "circle", "--num-robots", "4"),
    ),
    "mobile_circle_n8": BenchmarkCase(
        key="mobile_circle_n8",
        title="Mobile 2D circle, 8 robots",
        executable="mobile_robot_2d_crossing",
        base_args=("--scenario", "circle", "--num-robots", "8"),
    ),
    "planar_cross_n4": BenchmarkCase(
        key="planar_cross_n4",
        title="Planar manipulator cross, 4 robots",
        executable="planar_manipulator_cross",
        base_args=("--scenario", "cross", "--num-robots", "4"),
    ),
    "planar_cross_n8": BenchmarkCase(
        key="planar_cross_n8",
        title="Planar manipulator cross, 8 robots",
        executable="planar_manipulator_cross",
        base_args=("--scenario", "cross", "--num-robots", "8"),
    ),
    "planar_adaptive_n8": BenchmarkCase(
        key="planar_adaptive_n8",
        title="Planar manipulator adaptive, 8 robots",
        executable="planar_manipulator_cross",
        base_args=("--scenario", "adaptive", "--num-robots", "8"),
    ),
    "panda_cage_n2": BenchmarkCase(
        key="panda_cage_n2",
        title="Panda cage, 2 robots",
        executable="panda_cage",
        base_args=("--num-robots", "2"),
        task_based=True,
    ),
    "panda_cage_n4": BenchmarkCase(
        key="panda_cage_n4",
        title="Panda cage, 4 robots",
        executable="panda_cage",
        base_args=("--num-robots", "4"),
        task_based=True,
    ),
    "panda_flat_n4": BenchmarkCase(
        key="panda_flat_n4",
        title="Panda flat, 4 robots",
        executable="panda_flat",
        base_args=("--num-robots", "4"),
        task_based=True,
    ),
}

DEFAULT_FEASIBILITY_CASES = ("mobile_parallel_n4", "planar_cross_n4", "panda_cage_n2")
DEFAULT_ANYTIME_CASES = ("mobile_parallel_n4", "planar_cross_n4", "panda_cage_n2")
DEFAULT_MULTICORE_CASES = ("mobile_parallel_n8", "planar_cross_n8")

PLANNER_LABELS = {
    "arc": "ARC",
    "ao_arc": "AOARC",
    "parallel_arc": "ParallelARC",
    "composite": "CompositeRRT",
    "composite_rrtstar": "CompositeRRTStar",
    "composite_rrt_star": "CompositeRRTStar",
    "composite_prmstar": "CompositePRMStar",
    "composite_prm_star": "CompositePRMStar",
    "composite_aorrtc": "CompositeAORRTC",
    "cooperative_composite": "CooperativeCompositeRRT",
    "prioritized": "PrioritizedSTRRT",
    "drrt": "MRdRRT",
    "drrt_star": "MRdRRTStar",
    "ao_drrt": "MRdRRTStar",
    "stcbs": "STCBS",
}

RESULT_COLUMNS = [
    "case",
    "case_title",
    "task_index",
    "seed",
    "method",
    "time_limit_seconds",
    "success",
    "first_solution_time_seconds",
]

EVENT_COLUMNS = [
    "case",
    "task_index",
    "seed",
    "method",
    "elapsed_seconds",
    "makespan_timesteps",
]

METHOD_COLORS = [
    "#1b9e77",
    "#d95f02",
    "#7570b3",
    "#e7298a",
    "#66a61e",
    "#e6ab02",
    "#a6761d",
    "#666666",
]


def format_float(value: float) -> str:
    text = f"{value:.12g}"
    return text if "." in text or "e" in text.lower() else f"{text}.0"


def slug(value: Any) -> str:
    return "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in str(value))


def timestamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")


def parse_csv_tokens(value: str) -> list[str]:
    return [token.strip() for token in value.split(",") if token.strip()]


def parse_int_csv(value: str) -> list[int]:
    return [int(token) for token in parse_csv_tokens(value)]


def parse_cases(value: str, default_keys: Sequence[str]) -> list[BenchmarkCase]:
    keys = list(default_keys) if value == "default" else parse_csv_tokens(value)
    if not keys:
        raise RuntimeError("At least one benchmark case is required")
    cases: list[BenchmarkCase] = []
    for key in keys:
        try:
            cases.append(CASE_CATALOG[key])
        except KeyError as exc:
            available = ", ".join(sorted(CASE_CATALOG))
            raise RuntimeError(f"Unknown benchmark case '{key}'. Available: {available}") from exc
    return cases


def variants_from_algorithms(
    algorithms: str,
    *,
    extra_args_by_algorithm: dict[str, tuple[str, ...]] | None = None,
) -> list[PlannerVariant]:
    extras = extra_args_by_algorithm or {}
    variants: list[PlannerVariant] = []
    for algorithm in parse_csv_tokens(algorithms):
        label = PLANNER_LABELS.get(algorithm, algorithm)
        variants.append(
            PlannerVariant(
                label=label,
                algorithm=algorithm,
                slug=slug(algorithm),
                extra_args=extras.get(algorithm, ()),
            )
        )
    if not variants:
        raise RuntimeError("At least one planner algorithm is required")
    return variants


def multicore_variants(
    worker_counts: Sequence[int],
    *,
    parallel_arc_initial_solution_or: bool = False,
) -> list[PlannerVariant]:
    variants = [
        PlannerVariant(label="ARC", algorithm="arc", slug="arc"),
    ]
    for workers in worker_counts:
        if workers < 1:
            raise RuntimeError("Worker counts must be positive")
        variants.append(
            PlannerVariant(
                label=f"ParallelARC-{workers}",
                algorithm="parallel_arc",
                slug=f"parallel_arc_{workers}",
                extra_args=(
                    "--parallel-arc-worker-processes",
                    str(workers),
                    *(
                        ("--parallel-arc-initial-solution-or",)
                        if parallel_arc_initial_solution_or
                        else ()
                    ),
                ),
            )
        )
    return variants


def build_trial_specs(
    *,
    cases: Sequence[BenchmarkCase],
    variants: Sequence[PlannerVariant],
    seeds: Sequence[int],
    task_indices: Sequence[int],
    time_limit: float,
    collision_backend: str,
    resolution: int,
    build_dir: Path,
    output_root: Path,
) -> list[TrialSpec]:
    specs: list[TrialSpec] = []
    for case in cases:
        case_task_indices: Sequence[int | None] = task_indices if case.task_based else (None,)
        for task_index in case_task_indices:
            for seed in seeds:
                for variant in variants:
                    specs.append(
                        TrialSpec(
                            case=case,
                            variant=variant,
                            seed=seed,
                            task_index=task_index,
                            time_limit=time_limit,
                            collision_backend=collision_backend,
                            resolution=resolution,
                            build_dir=build_dir,
                            output_root=output_root,
                        )
                    )
    return specs


def finite_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def csv_scalar(value: Any) -> Any:
    return "" if value is None else value


def load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    try:
        with path.open() as handle:
            data = json.load(handle)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"{path} is not valid JSON") from exc
    return data if isinstance(data, dict) else {}


def normalized_solution_events(metrics: dict[str, Any]) -> list[dict[str, Any]]:
    planner_stats = metrics.get("planner_stats")
    if not isinstance(planner_stats, dict):
        planner_stats = {}
    raw_events = planner_stats.get("solution_events")
    if not isinstance(raw_events, list):
        raw_events = []

    events: list[dict[str, Any]] = []
    for raw_event in raw_events:
        if not isinstance(raw_event, dict):
            continue
        elapsed = finite_number(raw_event.get("elapsed_seconds"))
        makespan = finite_number(raw_event.get("makespan_timesteps"))
        if elapsed is None or makespan is None:
            continue
        events.append(
            {
                "elapsed_seconds": max(0.0, elapsed),
                "makespan_timesteps": raw_event.get("makespan_timesteps"),
            }
        )

    if not events and metrics.get("success") and metrics.get("makespan_timesteps") is not None:
        elapsed = finite_number(metrics.get("planning_time_seconds"))
        makespan = finite_number(metrics.get("makespan_timesteps"))
        if elapsed is not None and makespan is not None:
            events.append(
                {
                    "elapsed_seconds": max(0.0, elapsed),
                    "makespan_timesteps": metrics.get("makespan_timesteps"),
                }
            )

    events.sort(key=lambda item: float(item["elapsed_seconds"]))
    return events


def first_solution_event(metrics: dict[str, Any]) -> dict[str, Any] | None:
    events = normalized_solution_events(metrics)
    return events[0] if events else None


def run_trial(spec: TrialSpec, timeout_seconds: float | None) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    with tempfile.TemporaryDirectory(prefix="comotion-benchmark-") as tmpdir:
        metrics_path = Path(tmpdir) / "metrics.json"
        command = spec.command(metrics_path)

        timed_out = False
        returncode: int | None
        try:
            completed = subprocess.run(
                command,
                cwd=RUNTIME_CWD,
                capture_output=True,
                text=True,
                timeout=timeout_seconds,
                check=False,
            )
            returncode = completed.returncode
        except subprocess.TimeoutExpired:
            timed_out = True
            returncode = None

        metrics = load_json(metrics_path)

    result_row = build_result_row(
        spec=spec,
        metrics=metrics,
        returncode=returncode,
        timed_out=timed_out,
    )
    event_rows = build_event_rows(spec=spec, metrics=metrics)
    return result_row, event_rows


def build_result_row(
    *,
    spec: TrialSpec,
    metrics: dict[str, Any],
    returncode: int | None,
    timed_out: bool,
) -> dict[str, Any]:
    first = first_solution_event(metrics)
    return {
        "case": spec.case.key,
        "case_title": spec.case.title,
        "task_index": "" if spec.task_index is None else spec.task_index,
        "seed": spec.seed,
        "method": spec.variant.label,
        "time_limit_seconds": format_float(spec.time_limit),
        "success": bool(metrics.get("success")) and returncode == 0 and not timed_out,
        "first_solution_time_seconds": csv_scalar(
            first.get("elapsed_seconds") if first else None
        ),
    }


def build_event_rows(spec: TrialSpec, metrics: dict[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for event in normalized_solution_events(metrics):
        rows.append(
            {
                "case": spec.case.key,
                "task_index": "" if spec.task_index is None else spec.task_index,
                "seed": spec.seed,
                "method": spec.variant.label,
                "elapsed_seconds": event["elapsed_seconds"],
                "makespan_timesteps": event["makespan_timesteps"],
            }
        )
    return rows


def run_trials(
    specs: Sequence[TrialSpec],
    *,
    jobs: int,
    timeout_seconds: float | None,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    if jobs < 1:
        raise RuntimeError("--jobs must be at least 1")

    result_rows: list[dict[str, Any]] = []
    event_rows: list[dict[str, Any]] = []

    if jobs == 1:
        for index, spec in enumerate(specs, start=1):
            print(
                f"[{index}/{len(specs)}] {spec.case.key} {spec.variant.label} "
                f"seed={spec.seed} task={spec.task_label}",
                flush=True,
            )
            row, rows = run_trial(spec, timeout_seconds)
            result_rows.append(row)
            event_rows.extend(rows)
        return result_rows, event_rows

    import concurrent.futures

    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as pool:
        futures = {
            pool.submit(run_trial, spec, timeout_seconds): spec
            for spec in specs
        }
        completed_count = 0
        for future in concurrent.futures.as_completed(futures):
            spec = futures[future]
            completed_count += 1
            print(
                f"[{completed_count}/{len(specs)}] finished {spec.case.key} "
                f"{spec.variant.label} seed={spec.seed} task={spec.task_label}",
                flush=True,
            )
            row, rows = future.result()
            result_rows.append(row)
            event_rows.extend(rows)

    result_rows.sort(key=result_sort_key)
    event_rows.sort(key=event_sort_key)
    return result_rows, event_rows


def result_sort_key(row: dict[str, Any]) -> tuple[Any, ...]:
    return (
        row.get("case", ""),
        row.get("task_index", ""),
        int(row.get("seed", 0)),
        row.get("method", ""),
    )


def event_sort_key(row: dict[str, Any]) -> tuple[Any, ...]:
    return (
        row.get("case", ""),
        row.get("task_index", ""),
        int(row.get("seed", 0)),
        row.get("method", ""),
        float(row.get("elapsed_seconds", 0.0)),
    )


def write_csv(path: Path, columns: Sequence[str], rows: Iterable[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(columns))
        writer.writeheader()
        for row in rows:
            writer.writerow({column: row.get(column, "") for column in columns})


def write_manifest(
    output_root: Path,
    *,
    experiment_type: str,
    command_line: Sequence[str],
    cases: Sequence[BenchmarkCase],
    variants: Sequence[PlannerVariant],
    trial_count: int,
) -> None:
    manifest = {
        "schema": "comotion.benchmark_manifest.v1",
        "experiment_type": experiment_type,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command_line": list(command_line),
        "trial_count": trial_count,
        "cases": [
            {
                "key": case.key,
                "title": case.title,
                "executable": case.executable,
                "base_args": list(case.base_args),
                "task_based": case.task_based,
            }
            for case in cases
        ],
        "variants": [
            {
                "label": variant.label,
                "algorithm": variant.algorithm,
                "extra_args": list(variant.extra_args),
            }
            for variant in variants
        ],
    }
    output_root.mkdir(parents=True, exist_ok=True)
    (output_root / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


def import_pyplot(output_root: Path):
    try:
        global _MPL_CACHE_DIR
        if "MPLCONFIGDIR" not in os.environ:
            _MPL_CACHE_DIR = tempfile.TemporaryDirectory(
                prefix="comotion-matplotlib-"
            )
            os.environ["MPLCONFIGDIR"] = _MPL_CACHE_DIR.name

        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("warning: matplotlib is not installed; skipping plots", file=sys.stderr)
        return None
    return plt


def truthy(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "success"}


def float_or_none(value: Any) -> float | None:
    if value in ("", None):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def unique_in_order(values: Iterable[str]) -> list[str]:
    seen: set[str] = set()
    out: list[str] = []
    for value in values:
        if value not in seen:
            seen.add(value)
            out.append(value)
    return out


def group_rows_by_case(rows: Sequence[dict[str, Any]]) -> dict[str, list[dict[str, Any]]]:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault(str(row["case"]), []).append(row)
    return grouped


def method_color(methods: Sequence[str]) -> dict[str, str]:
    return {method: METHOD_COLORS[i % len(METHOD_COLORS)] for i, method in enumerate(methods)}


def first_solve_time(row: dict[str, Any]) -> float | None:
    if not truthy(row.get("success")):
        return None
    return float_or_none(row.get("first_solution_time_seconds"))


def write_success_plots(rows: Sequence[dict[str, Any]], output_root: Path) -> list[Path]:
    plt = import_pyplot(output_root)
    if plt is None:
        return []

    plot_dir = output_root / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []
    for case_key, case_rows in group_rows_by_case(rows).items():
        methods = unique_in_order(str(row["method"]) for row in case_rows)
        colors = method_color(methods)
        fig, ax = plt.subplots(figsize=(8.0, 5.0), dpi=160)
        x_max = max(float(row["time_limit_seconds"]) for row in case_rows)
        for method in methods:
            method_rows = [row for row in case_rows if row["method"] == method]
            solve_times = sorted(
                value
                for value in (first_solve_time(row) for row in method_rows)
                if value is not None
            )
            n_trials = len(method_rows)
            x_values = [0.0]
            y_values = [0.0]
            solved = 0
            for solve_time in solve_times:
                if solve_time > x_max:
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
                linewidth=2.4,
                color=colors[method],
                label=method,
            )
        title = str(case_rows[0].get("case_title", case_key))
        ax.set_title(title)
        ax.set_xlabel("Runtime (s)")
        ax.set_ylabel("Successful solves (%)")
        ax.set_xlim(0.0, x_max)
        ax.set_ylim(0.0, 100.0)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="lower right")
        fig.tight_layout()
        png = plot_dir / f"{case_key}_success.png"
        svg = plot_dir / f"{case_key}_success.svg"
        fig.savefig(png)
        fig.savefig(svg)
        plt.close(fig)
        written.extend([png, svg])
    return written


def latest_makespan_at(events: Sequence[dict[str, Any]], t: float) -> float | None:
    best: float | None = None
    for event in events:
        elapsed = float_or_none(event.get("elapsed_seconds"))
        makespan = float_or_none(event.get("makespan_timesteps"))
        if elapsed is None or makespan is None:
            continue
        if elapsed > t:
            break
        best = makespan
    return best


def write_anytime_panel_plots(
    rows: Sequence[dict[str, Any]],
    event_rows: Sequence[dict[str, Any]],
    output_root: Path,
) -> list[Path]:
    plt = import_pyplot(output_root)
    if plt is None:
        return []

    plot_dir = output_root / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []
    events_by_key: dict[tuple[str, str, str, str], list[dict[str, Any]]] = {}
    for event in event_rows:
        key = (
            str(event["case"]),
            str(event.get("task_index", "")),
            str(event["method"]),
            str(event["seed"]),
        )
        events_by_key.setdefault(key, []).append(event)
    for events in events_by_key.values():
        events.sort(key=lambda event: float(event["elapsed_seconds"]))

    for case_key, case_rows in group_rows_by_case(rows).items():
        methods = unique_in_order(str(row["method"]) for row in case_rows)
        colors = method_color(methods)
        x_max = max(float(row["time_limit_seconds"]) for row in case_rows)
        title = str(case_rows[0].get("case_title", case_key))
        fig, (ax_success, ax_makespan) = plt.subplots(
            2, 1, figsize=(8.0, 7.0), dpi=160, sharex=True
        )

        for method in methods:
            method_rows = [row for row in case_rows if row["method"] == method]
            solve_times = sorted(
                value
                for value in (first_solve_time(row) for row in method_rows)
                if value is not None
            )
            n_trials = len(method_rows)
            x_success = [0.0]
            y_success = [0.0]
            solved = 0
            for solve_time in solve_times:
                if solve_time > x_max:
                    continue
                x_success.extend([solve_time, solve_time])
                y_success.extend(
                    [100.0 * solved / n_trials, 100.0 * (solved + 1) / n_trials]
                )
                solved += 1
            x_success.append(x_max)
            y_success.append(100.0 * solved / n_trials)
            ax_success.step(
                x_success,
                y_success,
                where="post",
                linewidth=2.2,
                color=colors[method],
                label=method,
            )

            grid = {0.0, x_max}
            per_seed_events: list[list[dict[str, Any]]] = []
            for row in method_rows:
                key = (
                    str(row["case"]),
                    str(row.get("task_index", "")),
                    str(row["method"]),
                    str(row["seed"]),
                )
                seed_events = events_by_key.get(key, [])
                per_seed_events.append(seed_events)
                for event in seed_events:
                    elapsed = float_or_none(event.get("elapsed_seconds"))
                    if elapsed is not None and 0.0 <= elapsed <= x_max:
                        grid.add(elapsed)
            x_grid = sorted(grid)
            median_x: list[float] = []
            median_y: list[float] = []
            for t in x_grid:
                values = [
                    value
                    for value in (
                        latest_makespan_at(seed_events, t)
                        for seed_events in per_seed_events
                    )
                    if value is not None
                ]
                if values:
                    median_x.append(t)
                    median_y.append(statistics.median(values))
            if median_x:
                ax_makespan.step(
                    median_x,
                    median_y,
                    where="post",
                    linewidth=2.2,
                    color=colors[method],
                    label=method,
                )

        ax_success.set_title(title)
        ax_success.set_ylabel("Successful solves (%)")
        ax_success.set_ylim(0.0, 100.0)
        ax_success.grid(True, alpha=0.3)
        ax_success.legend(loc="lower right")
        ax_makespan.set_xlabel("Runtime (s)")
        ax_makespan.set_ylabel("Median current best makespan")
        ax_makespan.set_xlim(0.0, x_max)
        ax_makespan.grid(True, alpha=0.3)
        fig.tight_layout()
        png = plot_dir / f"{case_key}_anytime_panel.png"
        svg = plot_dir / f"{case_key}_anytime_panel.svg"
        fig.savefig(png)
        fig.savefig(svg)
        plt.close(fig)
        written.extend([png, svg])
    return written


def finish_outputs(
    *,
    output_root: Path,
    result_rows: Sequence[dict[str, Any]],
    event_rows: Sequence[dict[str, Any]],
    plot_kind: str,
) -> list[Path]:
    result_rows = sorted(result_rows, key=result_sort_key)
    event_rows = sorted(event_rows, key=event_sort_key)
    write_csv(output_root / "results.csv", RESULT_COLUMNS, result_rows)
    write_csv(output_root / "solution_events.csv", EVENT_COLUMNS, event_rows)
    if plot_kind == "success":
        return write_success_plots(result_rows, output_root)
    if plot_kind == "anytime":
        return write_anytime_panel_plots(result_rows, event_rows, output_root)
    return []
