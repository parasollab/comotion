from __future__ import annotations

import csv
import json
import math
import os
import signal
import statistics
import subprocess
import sys
import tempfile
import threading
import time
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
_CSV_WRITE_LOCK = threading.Lock()
PARALLEL_ARC_ASSIGNMENT_FLAG = "--parallel-arc-conflict-find-assignment"
DEFAULT_PARALLEL_ARC_CONFLICT_FIND_ASSIGNMENT = "cyclic_cover_greedy"


def effective_variant_extra_args(variant: "PlannerVariant") -> tuple[str, ...]:
    """Return launch arguments with the benchmark P-ARC defaults applied."""
    has_assignment = any(
        arg == PARALLEL_ARC_ASSIGNMENT_FLAG
        or arg.startswith(f"{PARALLEL_ARC_ASSIGNMENT_FLAG}=")
        for arg in variant.extra_args
    )
    if variant.algorithm != "parallel_arc" or has_assignment:
        return variant.extra_args
    return (
        PARALLEL_ARC_ASSIGNMENT_FLAG,
        DEFAULT_PARALLEL_ARC_CONFLICT_FIND_ASSIGNMENT,
        *variant.extra_args,
    )


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
        command.extend(effective_variant_extra_args(self.variant))
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
    "mobile_parallel_n32": BenchmarkCase(
        key="mobile_parallel_n32",
        title="Mobile 2D parallel, 32 robots",
        executable="mobile_robot_2d_crossing",
        base_args=("--scenario", "parallel", "--num-robots", "32"),
    ),
    "mobile_parallel_n64": BenchmarkCase(
        key="mobile_parallel_n64",
        title="Mobile 2D parallel, 64 robots",
        executable="mobile_robot_2d_crossing",
        base_args=("--scenario", "parallel", "--num-robots", "64"),
    ),
    "mobile_parallel_n128": BenchmarkCase(
        key="mobile_parallel_n128",
        title="Mobile 2D parallel, 128 robots",
        executable="mobile_robot_2d_crossing",
        base_args=("--scenario", "parallel", "--num-robots", "128"),
    ),
    "mobile_parallel_n256": BenchmarkCase(
        key="mobile_parallel_n256",
        title="Mobile 2D parallel, 256 robots",
        executable="mobile_robot_2d_crossing",
        base_args=("--scenario", "parallel", "--num-robots", "256"),
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
    "mobile_circle_n16": BenchmarkCase(
        key="mobile_circle_n16",
        title="Mobile 2D circle, 16 robots",
        executable="mobile_robot_2d_crossing",
        base_args=("--scenario", "circle", "--num-robots", "16"),
    ),
    "mobile_circle_n32": BenchmarkCase(
        key="mobile_circle_n32",
        title="Mobile 2D circle, 32 robots",
        executable="mobile_robot_2d_crossing",
        base_args=("--scenario", "circle", "--num-robots", "32"),
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
    "planar_cross_n16": BenchmarkCase(
        key="planar_cross_n16",
        title="Planar manipulator cross, 16 robots",
        executable="planar_manipulator_cross",
        base_args=("--scenario", "cross", "--num-robots", "16"),
    ),
    "planar_cross_n32": BenchmarkCase(
        key="planar_cross_n32",
        title="Planar manipulator cross, 32 robots",
        executable="planar_manipulator_cross",
        base_args=("--scenario", "cross", "--num-robots", "32"),
    ),
    "planar_cross_n64": BenchmarkCase(
        key="planar_cross_n64",
        title="Planar manipulator cross, 64 robots",
        executable="planar_manipulator_cross",
        base_args=("--scenario", "cross", "--num-robots", "64"),
    ),
    "planar_cross_n128": BenchmarkCase(
        key="planar_cross_n128",
        title="Planar manipulator cross, 128 robots",
        executable="planar_manipulator_cross",
        base_args=("--scenario", "cross", "--num-robots", "128"),
    ),
    "planar_cross_n256": BenchmarkCase(
        key="planar_cross_n256",
        title="Planar manipulator cross, 256 robots",
        executable="planar_manipulator_cross",
        base_args=("--scenario", "cross", "--num-robots", "256"),
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
    "panda_cage_n8": BenchmarkCase(
        key="panda_cage_n8",
        title="Panda cage, 8 robots",
        executable="panda_cage",
        base_args=("--num-robots", "8"),
        task_based=True,
    ),
    "panda_cage_n16": BenchmarkCase(
        key="panda_cage_n16",
        title="Panda cage, 16 robots",
        executable="panda_cage",
        base_args=("--num-robots", "16"),
        task_based=True,
    ),
    "panda_flat_n4": BenchmarkCase(
        key="panda_flat_n4",
        title="Panda flat, 4 robots",
        executable="panda_flat",
        base_args=("--num-robots", "4"),
        task_based=True,
    ),
    "heterogeneous_corridor_p4_s8": BenchmarkCase(
        key="heterogeneous_corridor_p4_s8",
        title="Heterogeneous corridor, 4 Pandas + 8 spheres",
        executable="heterogeneous_corridor",
        base_args=("--num-pandas", "4", "--num-spheres", "8"),
        task_based=True,
    ),
    "heterogeneous_corridor_p8_s16": BenchmarkCase(
        key="heterogeneous_corridor_p8_s16",
        title="Heterogeneous corridor, 8 Pandas + 16 spheres",
        executable="heterogeneous_corridor",
        base_args=("--num-pandas", "8", "--num-spheres", "16"),
        task_based=True,
    ),
    "heterogeneous_corridor_p16_s32": BenchmarkCase(
        key="heterogeneous_corridor_p16_s32",
        title="Heterogeneous corridor, 16 Pandas + 32 spheres",
        executable="heterogeneous_corridor",
        base_args=(
            "--num-pandas",
            "16",
            "--num-spheres",
            "32",
            "--panda-spacing",
            "1.5",
        ),
        task_based=True,
    ),
}

DEFAULT_FEASIBILITY_CASES = ("mobile_parallel_n4", "planar_cross_n4", "panda_cage_n2")
DEFAULT_ANYTIME_CASES = ("mobile_parallel_n4", "planar_cross_n4", "panda_cage_n2")
DEFAULT_MULTICORE_CASES = ("mobile_parallel_n8", "planar_cross_n8")

PAPER_MOBILE_PARALLEL_CASES = (
    "mobile_parallel_n4",
    "mobile_parallel_n8",
    "mobile_parallel_n16",
    "mobile_parallel_n32",
    "mobile_parallel_n64",
    "mobile_parallel_n128",
    "mobile_parallel_n256",
)
PAPER_MOBILE_CIRCLE_CASES = (
    "mobile_circle_n4",
    "mobile_circle_n8",
    "mobile_circle_n16",
)
PAPER_PLANAR_CROSS_CASES = (
    "planar_cross_n4",
    "planar_cross_n8",
    "planar_cross_n16",
    "planar_cross_n32",
    "planar_cross_n64",
    "planar_cross_n128",
    "planar_cross_n256",
)
PAPER_2D_CASES = (
    *PAPER_MOBILE_PARALLEL_CASES,
    *PAPER_MOBILE_CIRCLE_CASES,
    *PAPER_PLANAR_CROSS_CASES,
)
PAPER_PANDA_CAGE_CASES = (
    "panda_cage_n4",
    "panda_cage_n8",
    "panda_cage_n16",
)
PAPER_PARALLEL_ARC_CASES = (
    *PAPER_2D_CASES,
    *PAPER_PANDA_CAGE_CASES,
)
PAPER_CONFLICT_ABLATION_CASES = (
    "mobile_parallel_n64",
    "planar_cross_n64",
)
PAPER_OPTIMISTIC_CONFLICT_ABLATION_CASES = ("panda_cage_n8",)

CASE_GROUPS: dict[str, tuple[str, ...]] = {
    "paper": PAPER_PARALLEL_ARC_CASES,
    "paper_parallel_arc": PAPER_PARALLEL_ARC_CASES,
    "paper_2d": PAPER_2D_CASES,
    "paper_mobile": (*PAPER_MOBILE_PARALLEL_CASES, *PAPER_MOBILE_CIRCLE_CASES),
    "paper_mobile_parallel": PAPER_MOBILE_PARALLEL_CASES,
    "paper_mobile_cross": PAPER_MOBILE_PARALLEL_CASES,
    "paper_mobile_circle": PAPER_MOBILE_CIRCLE_CASES,
    "paper_planar": PAPER_PLANAR_CROSS_CASES,
    "paper_planar_cross": PAPER_PLANAR_CROSS_CASES,
    "paper_panda": PAPER_PANDA_CAGE_CASES,
    "paper_panda_cage": PAPER_PANDA_CAGE_CASES,
    "paper_conflict_ablation": PAPER_CONFLICT_ABLATION_CASES,
    "paper_optimistic_conflict_ablation": (
        PAPER_OPTIMISTIC_CONFLICT_ABLATION_CASES
    ),
}

PLANNER_LABELS = {
    "arc": "ARC",
    "ao_arc": "AOARC",
    "parallel_arc": "ParallelARC",
    "composite": "Composite RRT-C",
    "composite_rrtstar": "CompositeRRTStar",
    "composite_rrt_star": "CompositeRRTStar",
    "composite_prmstar": "CompositePRMStar",
    "composite_prm_star": "CompositePRMStar",
    "composite_aorrtc": "CompositeAORRTC",
    "cooperative_composite": "CooperativeCompositeRRT",
    "prioritized": "PP-ST-RRT",
    "drrt": "MR-dRRT",
    "drrt_star": "MRdRRTStar",
    "ao_drrt": "MRdRRTStar",
    "stcbs": "ST-CBS",
}

RESULT_COLUMNS = [
    "case",
    "case_title",
    "task_index",
    "seed",
    "method",
    "collision_backend",
    "time_limit_seconds",
    "success",
    "first_solution_time_seconds",
    "planning_time_seconds",
    "makespan_timesteps",
    "sum_of_cost_timesteps",
    "metrics_json",
]

EVENT_COLUMNS = [
    "case",
    "task_index",
    "seed",
    "method",
    "collision_backend",
    "elapsed_seconds",
    "makespan_timesteps",
]

METHOD_COLOR_BY_LABEL = {
    "ARC": "#7b2cbf",
    "Composite RRT-C": "#4169e1",
    "CompositeRRT": "#4169e1",
    "MR-dRRT": "#d4a017",
    "MRdRRT": "#d4a017",
    "ST-CBS": "#2e7d32",
    "STCBS": "#2e7d32",
    "PP-ST-RRT": "#d62728",
    "PrioritizedSTRRT": "#d62728",
}

FALLBACK_METHOD_COLORS = [
    "#1b9e77",
    "#d95f02",
    "#7570b3",
    "#e7298a",
    "#66a61e",
    "#e6ab02",
    "#a6761d",
    "#666666",
]

BACKEND_LINESTYLES = {
    "vamp": "-",
    "sphere": "--",
    "fcl": ":",
}

BACKEND_ORDER = ("vamp", "sphere", "fcl")

BACKEND_LABELS = {
    "vamp": "VAMP",
    "sphere": "Sphere",
    "fcl": "FCL",
}


def format_float(value: float) -> str:
    text = f"{value:.12g}"
    return text if "." in text or "e" in text.lower() else f"{text}.0"


def slug(value: Any) -> str:
    return "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in str(value))


def timestamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")


def resolve_output_paths(
    *,
    output_root: Path | None,
    results_csv: Path | None,
    solution_events_csv: Path | None,
    default_output_root: Path,
) -> tuple[Path, Path, Path]:
    if output_root is None:
        if results_csv is not None:
            output_root = results_csv.parent
        elif solution_events_csv is not None:
            output_root = solution_events_csv.parent
        else:
            output_root = default_output_root

    result_csv_path = results_csv or output_root / "results.csv"
    event_csv_path = (
        solution_events_csv
        or (result_csv_path.with_name("solution_events.csv")
            if results_csv is not None
            else output_root / "solution_events.csv")
    )
    return output_root, result_csv_path, event_csv_path


def parse_csv_tokens(value: str) -> list[str]:
    return [token.strip() for token in value.split(",") if token.strip()]


def parse_int_csv(value: str) -> list[int]:
    return [int(token) for token in parse_csv_tokens(value)]


def parse_cases(value: str, default_keys: Sequence[str]) -> list[BenchmarkCase]:
    raw_keys = list(default_keys) if value == "default" else parse_csv_tokens(value)
    keys: list[str] = []
    seen: set[str] = set()
    for raw_key in raw_keys:
        expanded = CASE_GROUPS.get(raw_key, (raw_key,))
        for key in expanded:
            if key not in seen:
                keys.append(key)
                seen.add(key)
    if not keys:
        raise RuntimeError("At least one benchmark case is required")
    cases: list[BenchmarkCase] = []
    for key in keys:
        try:
            cases.append(CASE_CATALOG[key])
        except KeyError as exc:
            available = ", ".join(
                [*sorted(CASE_CATALOG), *sorted(CASE_GROUPS)]
            )
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


PAPER_PARALLEL_ARC_WORKER_COUNTS = (2, 4, 8, 16)
PAPER_PARALLEL_ARC_TOTAL_WORKERS = 16
PAPER_OR_PP_ST_RRT_WORKER_COUNTS = (16,)
PAPER_OR_PARALLEL_SPLITS = ((2, 8), (4, 4), (8, 2))
PAPER_CONFLICT_FIND_HORIZONS = (50, 100, 200, 400, 800, 1600, 3200)


def paper_parallel_arc_variants(
    *,
    include_rrt_baselines: bool = False,
    include_initial_solution_or: bool = True,
    or_pp_strrt_worker_counts: Sequence[int] = PAPER_OR_PP_ST_RRT_WORKER_COUNTS,
) -> list[PlannerVariant]:
    initial_or_args = (
        ("--parallel-arc-initial-solution-or",)
        if include_initial_solution_or
        else ()
    )
    variants: list[PlannerVariant] = [
        PlannerVariant(label="ARC", algorithm="arc", slug="arc"),
    ]
    for workers in PAPER_PARALLEL_ARC_WORKER_COUNTS:
        variants.append(
            PlannerVariant(
                label=f"P-ARC-{workers}",
                algorithm="parallel_arc",
                slug=f"p_arc_{workers}",
                extra_args=(
                    "--parallel-arc-worker-processes",
                    str(workers),
                    *initial_or_args,
                ),
            )
        )

    variants.append(
        PlannerVariant(
            label=f"OR-ARC-{PAPER_PARALLEL_ARC_TOTAL_WORKERS}",
            algorithm="arc",
            slug=f"or_arc_{PAPER_PARALLEL_ARC_TOTAL_WORKERS}",
            extra_args=(
                "--or-parallel-worker-processes",
                str(PAPER_PARALLEL_ARC_TOTAL_WORKERS),
            ),
        )
    )
    for outer_workers, inner_workers in PAPER_OR_PARALLEL_SPLITS:
        variants.append(
            PlannerVariant(
                label=f"OR-P-ARC-{outer_workers}x{inner_workers}",
                algorithm="parallel_arc",
                slug=f"or_p_arc_{outer_workers}x{inner_workers}",
                extra_args=(
                    "--or-parallel-worker-processes",
                    str(outer_workers),
                    "--parallel-arc-worker-processes",
                    str(inner_workers),
                    *initial_or_args,
                ),
            )
        )

    if include_rrt_baselines:
        variants.extend(paper_parallel_rrt_baseline_variants())
        variants.extend(
            paper_or_pp_strrt_baseline_variants(or_pp_strrt_worker_counts)
        )
    return variants


def paper_or_pp_strrt_baseline_variants(
    worker_counts: Sequence[int] = PAPER_OR_PP_ST_RRT_WORKER_COUNTS,
) -> list[PlannerVariant]:
    variants: list[PlannerVariant] = []
    for workers in worker_counts:
        if workers < 1:
            raise RuntimeError("OR-PP-ST-RRT worker counts must be positive")
        variants.append(
            PlannerVariant(
                label=f"OR-PP-ST-RRT-{workers}",
                algorithm="prioritized",
                slug=f"or_pp_st_rrt_{workers}",
                extra_args=(
                    "--or-parallel-worker-processes",
                    str(workers),
                    "--strrt-shuffle-priority-order",
                ),
            )
        )
    return variants


def paper_parallel_rrt_baseline_variants() -> list[PlannerVariant]:
    variants: list[PlannerVariant] = [
        PlannerVariant(
            label="RRT-C",
            algorithm="composite",
            slug="rrt_c",
        ),
        PlannerVariant(
            label=f"C-RRT-C-{PAPER_PARALLEL_ARC_TOTAL_WORKERS}",
            algorithm="cooperative_composite",
            slug=f"c_rrt_c_{PAPER_PARALLEL_ARC_TOTAL_WORKERS}",
            extra_args=(
                "--cooperative-rrt-worker-threads",
                str(PAPER_PARALLEL_ARC_TOTAL_WORKERS),
            ),
        ),
        PlannerVariant(
            label=f"OR-RRT-C-{PAPER_PARALLEL_ARC_TOTAL_WORKERS}",
            algorithm="composite",
            slug=f"or_rrt_c_{PAPER_PARALLEL_ARC_TOTAL_WORKERS}",
            extra_args=(
                "--or-parallel-worker-processes",
                str(PAPER_PARALLEL_ARC_TOTAL_WORKERS),
            ),
        ),
    ]
    for outer_workers, inner_workers in PAPER_OR_PARALLEL_SPLITS:
        variants.append(
            PlannerVariant(
                label=f"OR-C-RRT-C-{outer_workers}x{inner_workers}",
                algorithm="cooperative_composite",
                slug=f"or_c_rrt_c_{outer_workers}x{inner_workers}",
                extra_args=(
                    "--or-parallel-worker-processes",
                    str(outer_workers),
                    "--cooperative-rrt-worker-threads",
                    str(inner_workers),
                ),
            )
        )
    return variants


def paper_conflict_horizon_variants() -> list[PlannerVariant]:
    return [
        PlannerVariant(
            label=f"P-ARC-16-h{horizon}",
            algorithm="parallel_arc",
            slug=f"p_arc_16_h{horizon}",
            extra_args=(
                "--parallel-arc-worker-processes",
                str(PAPER_PARALLEL_ARC_TOTAL_WORKERS),
                "--parallel-arc-initial-solution-or",
                PARALLEL_ARC_ASSIGNMENT_FLAG,
                "round_robin",
                "--parallel-arc-conflict-find-horizon",
                str(horizon),
                "--parallel-arc-conflict-ablation-only",
            ),
        )
        for horizon in PAPER_CONFLICT_FIND_HORIZONS
    ]


def paper_optimistic_conflict_ablation_variants() -> list[PlannerVariant]:
    base_args = (
        "--parallel-arc-worker-processes",
        str(PAPER_PARALLEL_ARC_TOTAL_WORKERS),
        "--parallel-arc-initial-solution-or",
    )
    return [
        PlannerVariant(
            label="P-ARC-16-optimistic",
            algorithm="parallel_arc",
            slug="p_arc_16_optimistic",
            extra_args=(
                *base_args,
                "--parallel-arc-conflict-batch-mode",
                "optimistic",
            ),
        ),
        PlannerVariant(
            label="P-ARC-16-independent-only",
            algorithm="parallel_arc",
            slug="p_arc_16_independent_only",
            extra_args=(
                *base_args,
                "--parallel-arc-conflict-batch-mode",
                "independent_only",
            ),
        ),
    ]


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


def metrics_output_path(spec: TrialSpec) -> Path:
    return (
        spec.output_root
        / "metrics"
        / spec.case.key
        / spec.variant.slug
        / spec.task_label
        / f"seed_{spec.seed}.json"
    )


def run_command_with_process_group_timeout(
    command: Sequence[str],
    *,
    timeout_seconds: float | None,
) -> tuple[int | None, bool]:
    """Run one trial and terminate its complete process tree on timeout.

    Parallel planners fork worker processes.  ``subprocess.run(timeout=...)``
    kills only the immediate app process, allowing its workers to become
    orphans and consume resources during later trials.  A fresh POSIX session
    gives every trial its own process group that can be terminated as a unit.
    """
    process = subprocess.Popen(
        list(command),
        cwd=RUNTIME_CWD,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=(os.name == "posix"),
    )
    def process_group_exists() -> bool:
        if os.name != "posix":
            return process.poll() is None
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            return False
        return True

    def wait_for_process_group_exit(seconds: float) -> bool:
        deadline = time.monotonic() + seconds
        while process_group_exists() and time.monotonic() < deadline:
            time.sleep(0.01)
        return not process_group_exists()

    def ensure_process_group_cleanup() -> None:
        if wait_for_process_group_exit(0.25):
            return
        if os.name == "posix":
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                return
        else:
            process.terminate()
        if wait_for_process_group_exit(1.0):
            return
        if os.name == "posix":
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                return
        else:
            process.kill()
        if not wait_for_process_group_exit(1.0):
            raise RuntimeError(
                f"trial process group {process.pid} did not terminate"
            )

    try:
        process.communicate(timeout=timeout_seconds)
        ensure_process_group_cleanup()
        return process.returncode, False
    except subprocess.TimeoutExpired:
        if os.name == "posix":
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        else:
            process.terminate()

        try:
            process.communicate(timeout=1.0)
        except subprocess.TimeoutExpired:
            if os.name == "posix":
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            else:
                process.kill()
            process.communicate()
        ensure_process_group_cleanup()
        return None, True


def run_trial(
    spec: TrialSpec,
    timeout_seconds: float | None,
    *,
    keep_metrics_json: bool = False,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    if keep_metrics_json:
        metrics_path = metrics_output_path(spec)
        metrics_path.parent.mkdir(parents=True, exist_ok=True)
        if metrics_path.exists():
            metrics_path.unlink()
        command = spec.command(metrics_path)

        timed_out = False
        returncode: int | None
        returncode, timed_out = run_command_with_process_group_timeout(
            command, timeout_seconds=timeout_seconds
        )

        metrics = load_json(metrics_path)
        result_row = build_result_row(
            spec=spec,
            metrics=metrics,
            returncode=returncode,
            timed_out=timed_out,
            metrics_path=metrics_path,
        )
        event_rows = build_event_rows(spec=spec, metrics=metrics)
        return result_row, event_rows

    with tempfile.TemporaryDirectory(prefix="comotion-benchmark-") as tmpdir:
        metrics_path = Path(tmpdir) / "metrics.json"
        command = spec.command(metrics_path)

        timed_out = False
        returncode: int | None
        returncode, timed_out = run_command_with_process_group_timeout(
            command, timeout_seconds=timeout_seconds
        )

        metrics = load_json(metrics_path)

    result_row = build_result_row(
        spec=spec,
        metrics=metrics,
        returncode=returncode,
        timed_out=timed_out,
        metrics_path=None,
    )
    event_rows = build_event_rows(spec=spec, metrics=metrics)
    return result_row, event_rows


def build_result_row(
    *,
    spec: TrialSpec,
    metrics: dict[str, Any],
    returncode: int | None,
    timed_out: bool,
    metrics_path: Path | None,
) -> dict[str, Any]:
    first = first_solution_event(metrics)
    return {
        "case": spec.case.key,
        "case_title": spec.case.title,
        "task_index": "" if spec.task_index is None else spec.task_index,
        "seed": spec.seed,
        "method": spec.variant.label,
        "collision_backend": spec.collision_backend,
        "time_limit_seconds": format_float(spec.time_limit),
        "success": bool(metrics.get("success")) and returncode == 0 and not timed_out,
        "first_solution_time_seconds": csv_scalar(
            first.get("elapsed_seconds") if first else None
        ),
        "planning_time_seconds": csv_scalar(metrics.get("planning_time_seconds")),
        "makespan_timesteps": csv_scalar(metrics.get("makespan_timesteps")),
        "sum_of_cost_timesteps": csv_scalar(metrics.get("sum_of_cost_timesteps")),
        "metrics_json": "" if metrics_path is None else str(metrics_path),
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
                "collision_backend": spec.collision_backend,
                "elapsed_seconds": event["elapsed_seconds"],
                "makespan_timesteps": event["makespan_timesteps"],
            }
        )
    return rows


def build_event_rows_from_result_row(
    result_row: dict[str, Any],
    metrics: dict[str, Any],
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for event in normalized_solution_events(metrics):
        rows.append(
            {
                "case": result_row.get("case", ""),
                "task_index": result_row.get("task_index", ""),
                "seed": result_row.get("seed", ""),
                "method": result_row.get("method", ""),
                "elapsed_seconds": event["elapsed_seconds"],
                "makespan_timesteps": event["makespan_timesteps"],
            }
        )
    return rows


def trial_identity_from_values(
    case: Any,
    task_index: Any,
    seed: Any,
    method: Any,
) -> tuple[str, str, str, str]:
    return (str(case), str(task_index), str(seed), str(method))


def result_trial_key(row: dict[str, Any]) -> tuple[str, str, str, str, str]:
    return (
        str(row.get("case", "")),
        str(row.get("task_index", "")),
        str(row.get("seed", "")),
        str(row.get("method", "")),
        str(row.get("time_limit_seconds", "")),
    )


def spec_trial_key(spec: TrialSpec) -> tuple[str, str, str, str, str]:
    return (
        spec.case.key,
        "" if spec.task_index is None else str(spec.task_index),
        str(spec.seed),
        spec.variant.label,
        format_float(spec.time_limit),
    )


def event_identity(row: dict[str, Any]) -> tuple[str, str, str, str]:
    return trial_identity_from_values(
        row.get("case", ""),
        row.get("task_index", ""),
        row.get("seed", ""),
        row.get("method", ""),
    )


def load_csv_rows(path: Path) -> list[dict[str, Any]]:
    if not path.is_file() or path.stat().st_size == 0:
        return []
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            return []
        return [dict(row) for row in reader]


def recover_event_rows_from_metrics(
    result_rows: Sequence[dict[str, Any]],
    event_rows: Sequence[dict[str, Any]],
) -> list[dict[str, Any]]:
    events = list(event_rows)
    event_identities = {event_identity(row) for row in events}
    for result_row in result_rows:
        identity = trial_identity_from_values(
            result_row.get("case", ""),
            result_row.get("task_index", ""),
            result_row.get("seed", ""),
            result_row.get("method", ""),
        )
        if identity in event_identities:
            continue
        metrics_json = str(result_row.get("metrics_json", "")).strip()
        if not metrics_json:
            continue
        recovered_rows = build_event_rows_from_result_row(
            result_row,
            load_json(Path(metrics_json)),
        )
        if recovered_rows:
            events.extend(recovered_rows)
            event_identities.add(identity)
    return events


def result_row_is_complete(
    row: dict[str, Any],
    *,
    keep_metrics_json: bool,
) -> bool:
    if not keep_metrics_json:
        return True
    metrics_json = str(row.get("metrics_json", "")).strip()
    planning_time = str(row.get("planning_time_seconds", "")).strip()
    if not metrics_json:
        return False

    metrics_path = Path(metrics_json)
    if metrics_path.is_file() and planning_time:
        return True

    # Some failed benchmark runs do not emit a metrics JSON at all. In that case
    # the CSV row is still the authoritative record that the trial finished and
    # failed, so do not keep retrying it forever just because the metrics file is
    # absent.
    return not truthy(row.get("success"))


def run_trials(
    specs: Sequence[TrialSpec],
    *,
    jobs: int,
    timeout_seconds: float | None,
    keep_metrics_json: bool = False,
    result_csv_path: Path | None = None,
    event_csv_path: Path | None = None,
    skip_existing: bool = True,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    if jobs < 1:
        raise RuntimeError("--jobs must be at least 1")

    if not skip_existing:
        if result_csv_path is not None:
            write_csv(result_csv_path, RESULT_COLUMNS, [])
        if event_csv_path is not None:
            write_csv(event_csv_path, EVENT_COLUMNS, [])

    result_rows: list[dict[str, Any]] = (
        load_csv_rows(result_csv_path)
        if skip_existing and result_csv_path is not None
        else []
    )
    event_rows: list[dict[str, Any]] = (
        load_csv_rows(event_csv_path)
        if skip_existing and event_csv_path is not None
        else []
    )
    if skip_existing and keep_metrics_json and result_rows:
        complete_rows = [
            row for row in result_rows
            if result_row_is_complete(row, keep_metrics_json=keep_metrics_json)
        ]
        incomplete_rows = [
            row for row in result_rows
            if not result_row_is_complete(row, keep_metrics_json=keep_metrics_json)
        ]
        if incomplete_rows:
            incomplete_identities = {
                trial_identity_from_values(
                    row.get("case", ""),
                    row.get("task_index", ""),
                    row.get("seed", ""),
                    row.get("method", ""),
                )
                for row in incomplete_rows
            }
            event_rows = [
                row for row in event_rows
                if event_identity(row) not in incomplete_identities
            ]
            result_rows = complete_rows
            if result_csv_path is not None:
                write_csv(result_csv_path, RESULT_COLUMNS, result_rows)
            if event_csv_path is not None:
                write_csv(event_csv_path, EVENT_COLUMNS, event_rows)
            print(
                f"rerunning {len(incomplete_rows)} incomplete trial row(s) "
                "with missing metrics JSON",
                flush=True,
            )
    event_rows = recover_event_rows_from_metrics(result_rows, event_rows)
    completed_keys = {result_trial_key(row) for row in result_rows}
    pending_specs = [
        spec for spec in specs
        if not skip_existing or spec_trial_key(spec) not in completed_keys
    ]
    skipped_count = len(specs) - len(pending_specs)
    if skipped_count:
        print(f"skipping {skipped_count} completed trial(s)", flush=True)

    if event_csv_path is not None and skip_existing:
        existing_event_keys = {event_sort_key(row) for row in load_csv_rows(event_csv_path)}
        recovered_event_rows = [
            row for row in event_rows
            if event_sort_key(row) not in existing_event_keys
        ]
        append_csv_rows(event_csv_path, EVENT_COLUMNS, recovered_event_rows)

    def record_finished_trial(
        row: dict[str, Any],
        rows: Sequence[dict[str, Any]],
    ) -> None:
        result_rows.append(row)
        event_rows.extend(rows)
        if result_csv_path is not None:
            append_csv_rows(result_csv_path, RESULT_COLUMNS, [row])
        if event_csv_path is not None:
            append_csv_rows(event_csv_path, EVENT_COLUMNS, rows)

    if not pending_specs:
        return result_rows, event_rows

    if jobs == 1:
        for index, spec in enumerate(pending_specs, start=1):
            print(
                f"[{index}/{len(pending_specs)}] {spec.case.key} {spec.variant.label} "
                f"seed={spec.seed} task={spec.task_label}",
                flush=True,
            )
            row, rows = run_trial(
                spec,
                timeout_seconds,
                keep_metrics_json=keep_metrics_json,
            )
            record_finished_trial(row, rows)
        return result_rows, event_rows

    import concurrent.futures

    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as pool:
        futures = {
            pool.submit(
                run_trial,
                spec,
                timeout_seconds,
                keep_metrics_json=keep_metrics_json,
            ): spec
            for spec in pending_specs
        }
        completed_count = 0
        for future in concurrent.futures.as_completed(futures):
            spec = futures[future]
            completed_count += 1
            print(
                f"[{completed_count}/{len(pending_specs)}] finished {spec.case.key} "
                f"{spec.variant.label} seed={spec.seed} task={spec.task_label}",
                flush=True,
            )
            row, rows = future.result()
            record_finished_trial(row, rows)

    result_rows.sort(key=result_sort_key)
    event_rows.sort(key=event_sort_key)
    return result_rows, event_rows


def result_sort_key(row: dict[str, Any]) -> tuple[Any, ...]:
    return (
        str(row.get("case", "")),
        str(row.get("task_index", "")),
        int(row.get("seed", 0)),
        str(row.get("method", "")),
        str(row.get("collision_backend", "")),
    )


def event_sort_key(row: dict[str, Any]) -> tuple[Any, ...]:
    return (
        str(row.get("case", "")),
        str(row.get("task_index", "")),
        int(row.get("seed", 0)),
        str(row.get("method", "")),
        float(row.get("elapsed_seconds", 0.0)),
    )


def _lock_file(handle: Any) -> None:
    try:
        import fcntl
    except ImportError:
        return
    fcntl.flock(handle.fileno(), fcntl.LOCK_EX)


def _unlock_file(handle: Any) -> None:
    try:
        import fcntl
    except ImportError:
        return
    fcntl.flock(handle.fileno(), fcntl.LOCK_UN)


def _write_csv_rows(
    writer: csv.DictWriter,
    columns: Sequence[str],
    rows: Iterable[dict[str, Any]],
) -> None:
    for row in rows:
        writer.writerow({column: row.get(column, "") for column in columns})


def append_csv_rows(
    path: Path,
    columns: Sequence[str],
    rows: Iterable[dict[str, Any]],
) -> None:
    rows = list(rows)
    if not rows:
        return

    path.parent.mkdir(parents=True, exist_ok=True)
    columns = list(columns)
    with _CSV_WRITE_LOCK:
        with path.open("a+", newline="") as handle:
            _lock_file(handle)
            try:
                handle.seek(0)
                fieldnames = next(csv.reader(handle), None)
                if fieldnames is not None:
                    if fieldnames != columns:
                        if not all(field in columns for field in fieldnames):
                            raise RuntimeError(
                                f"{path} has unsupported columns: "
                                f"{', '.join(fieldnames)}"
                            )
                        handle.seek(0)
                        existing_rows = [dict(row) for row in csv.DictReader(handle)]
                        handle.seek(0)
                        handle.truncate(0)
                        writer = csv.DictWriter(handle, fieldnames=columns)
                        writer.writeheader()
                        _write_csv_rows(writer, columns, existing_rows)
                else:
                    handle.seek(0)
                    handle.truncate(0)
                    writer = csv.DictWriter(handle, fieldnames=columns)
                    writer.writeheader()

                handle.seek(0, os.SEEK_END)
                if handle.tell() == 0:
                    writer = csv.DictWriter(handle, fieldnames=columns)
                    writer.writeheader()
                else:
                    writer = csv.DictWriter(handle, fieldnames=columns)
                _write_csv_rows(writer, columns, rows)
                handle.flush()
                os.fsync(handle.fileno())
            finally:
                _unlock_file(handle)


def write_csv(path: Path, columns: Sequence[str], rows: Iterable[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    with tmp_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(columns))
        writer.writeheader()
        _write_csv_rows(writer, columns, rows)
    tmp_path.replace(path)


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
                "extra_args": list(effective_variant_extra_args(variant)),
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


def display_method_label(value: Any) -> str:
    text = str(value).strip()
    return {
        "CompositeRRT": "Composite RRT-C",
        "PrioritizedSTRRT": "PP-ST-RRT",
        "MRdRRT": "MR-dRRT",
        "STCBS": "ST-CBS",
    }.get(text, text)


def row_method(row: dict[str, Any]) -> str:
    return display_method_label(row.get("method", ""))


def method_color(methods: Sequence[str]) -> dict[str, str]:
    colors: dict[str, str] = {}
    fallback_index = 0
    for method in methods:
        color = METHOD_COLOR_BY_LABEL.get(method)
        if color is None:
            color = FALLBACK_METHOD_COLORS[
                fallback_index % len(FALLBACK_METHOD_COLORS)
            ]
            fallback_index += 1
        colors[method] = color
    return colors


def row_backend(row: dict[str, Any]) -> str:
    value = str(row.get("collision_backend", "")).strip()
    return value or "vamp"


def ordered_backends(rows: Sequence[dict[str, Any]]) -> list[str]:
    backends = unique_in_order(row_backend(row) for row in rows)
    order = {backend: index for index, backend in enumerate(BACKEND_ORDER)}
    return sorted(
        backends,
        key=lambda backend: (
            order.get(backend.lower(), len(order)),
            backends.index(backend),
        ),
    )


def backend_label(backend: str) -> str:
    return BACKEND_LABELS.get(backend.lower(), backend)


def backend_linestyle(backend: str) -> str:
    return BACKEND_LINESTYLES.get(backend.lower(), ":")


def add_success_legends(
    ax: Any,
    *,
    methods: Sequence[str],
    colors: dict[str, str],
    backends: Sequence[str],
    plot_backends: bool,
) -> None:
    if not plot_backends:
        ax.legend(loc="lower right")
        return

    from matplotlib.lines import Line2D

    algorithm_handles = [
        Line2D([0], [0], color=colors[method], linestyle="-", linewidth=2.4, label=method)
        for method in methods
    ]
    backend_handles = [
        Line2D(
            [0],
            [0],
            color="#333333",
            linestyle=backend_linestyle(backend),
            linewidth=2.4,
            label=backend_label(backend),
        )
        for backend in backends
    ]
    algorithm_legend = ax.legend(
        handles=algorithm_handles,
        title="Algorithms",
        loc="lower right",
    )
    ax.add_artist(algorithm_legend)
    ax.legend(
        handles=backend_handles,
        title="Validation",
        loc="lower left",
    )


def first_solve_time(row: dict[str, Any]) -> float | None:
    if not truthy(row.get("success")):
        return None
    return float_or_none(row.get("first_solution_time_seconds"))


def log_runtime_lower_bound(
    rows: Sequence[dict[str, Any]],
    event_rows: Sequence[dict[str, Any]],
    x_max: float,
) -> float:
    positives: list[float] = []
    for row in rows:
        solve_time = first_solve_time(row)
        if solve_time is not None and solve_time > 0.0:
            positives.append(solve_time)
    for event in event_rows:
        elapsed = float_or_none(event.get("elapsed_seconds"))
        if elapsed is not None and elapsed > 0.0:
            positives.append(elapsed)

    if positives:
        lower = max(min(positives) / 10.0, 1e-6)
    else:
        lower = max(x_max / 1000.0, 1e-6)
    if lower >= x_max:
        lower = max(x_max / 1000.0, 1e-6)
    if lower >= x_max:
        lower = max(x_max * 0.1, 1e-6)
    return lower


def log_runtime_value(value: float, x_min: float) -> float:
    return value if value > x_min else x_min


def time_limit_seconds(row: dict[str, Any]) -> float:
    value = float_or_none(row.get("time_limit_seconds"))
    if value is None or value <= 0.0:
        raise RuntimeError("Plot rows must contain a positive time_limit_seconds value")
    return value


def log_runtime_lower_bounds_by_x_max(
    rows: Sequence[dict[str, Any]],
) -> dict[float, float]:
    rows_by_x_max: dict[float, list[dict[str, Any]]] = {}
    for row in rows:
        rows_by_x_max.setdefault(time_limit_seconds(row), []).append(row)
    return {
        x_max: log_runtime_lower_bound(x_max_rows, (), x_max)
        for x_max, x_max_rows in rows_by_x_max.items()
    }


def write_success_plots(
    rows: Sequence[dict[str, Any]],
    output_root: Path,
    *,
    plot_backends: bool = False,
) -> list[Path]:
    plt = import_pyplot(output_root)
    if plt is None:
        return []

    plot_dir = output_root / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []
    x_min_by_x_max = log_runtime_lower_bounds_by_x_max(rows)
    for case_key, case_rows in group_rows_by_case(rows).items():
        methods = unique_in_order(row_method(row) for row in case_rows)
        backends = ordered_backends(case_rows)
        colors = method_color(methods)
        fig, ax = plt.subplots(figsize=(8.0, 5.0), dpi=160)
        x_max = max(time_limit_seconds(row) for row in case_rows)
        x_min = x_min_by_x_max[x_max]
        for method in methods:
            method_backends = backends if plot_backends else [""]
            for backend in method_backends:
                method_rows = [
                    row
                    for row in case_rows
                    if row_method(row) == method
                    and (not plot_backends or row_backend(row) == backend)
                ]
                if not method_rows:
                    continue
                solve_times = sorted(
                    value
                    for value in (first_solve_time(row) for row in method_rows)
                    if value is not None
                )
                n_trials = len(method_rows)
                x_values = [x_min]
                y_values = [0.0]
                solved = 0
                for solve_time in solve_times:
                    if solve_time > x_max:
                        continue
                    solve_time = log_runtime_value(solve_time, x_min)
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
                    linestyle=backend_linestyle(backend) if plot_backends else "-",
                    label=method if not plot_backends else "_nolegend_",
                )
        title = str(case_rows[0].get("case_title", case_key))
        ax.set_title(title)
        ax.set_xlabel("Runtime (s)")
        ax.set_ylabel("Successful solves (%)")
        ax.set_xscale("log")
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(0.0, 100.0)
        ax.grid(True, alpha=0.3)
        add_success_legends(
            ax,
            methods=methods,
            colors=colors,
            backends=backends,
            plot_backends=plot_backends,
        )
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
    *,
    plot_backends: bool = False,
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
            row_backend(event),
        )
        events_by_key.setdefault(key, []).append(event)
    for events in events_by_key.values():
        events.sort(key=lambda event: float(event["elapsed_seconds"]))

    x_min_by_x_max = log_runtime_lower_bounds_by_x_max(rows)
    for case_key, case_rows in group_rows_by_case(rows).items():
        methods = unique_in_order(row_method(row) for row in case_rows)
        backends = ordered_backends(case_rows)
        colors = method_color(methods)
        x_max = max(time_limit_seconds(row) for row in case_rows)
        x_min = x_min_by_x_max[x_max]
        title = str(case_rows[0].get("case_title", case_key))
        fig, (ax_success, ax_makespan) = plt.subplots(
            2, 1, figsize=(8.0, 7.0), dpi=160, sharex=True
        )

        for method in methods:
            method_backends = backends if plot_backends else [""]
            for backend in method_backends:
                method_rows = [
                    row
                    for row in case_rows
                    if row_method(row) == method
                    and (not plot_backends or row_backend(row) == backend)
                ]
                if not method_rows:
                    continue
                solve_times = sorted(
                    value
                    for value in (first_solve_time(row) for row in method_rows)
                    if value is not None
                )
                n_trials = len(method_rows)
                x_success = [x_min]
                y_success = [0.0]
                solved = 0
                for solve_time in solve_times:
                    if solve_time > x_max:
                        continue
                    solve_time = log_runtime_value(solve_time, x_min)
                    x_success.extend([solve_time, solve_time])
                    y_success.extend(
                        [100.0 * solved / n_trials, 100.0 * (solved + 1) / n_trials]
                    )
                    solved += 1
                x_success.append(x_max)
                y_success.append(100.0 * solved / n_trials)
                linestyle = backend_linestyle(backend) if plot_backends else "-"
                ax_success.step(
                    x_success,
                    y_success,
                    where="post",
                    linewidth=2.2,
                    color=colors[method],
                    linestyle=linestyle,
                    label=method if not plot_backends else "_nolegend_",
                )

                grid = {x_min, x_max}
                per_seed_events: list[list[dict[str, Any]]] = []
                for row in method_rows:
                    key = (
                        str(row["case"]),
                        str(row.get("task_index", "")),
                        str(row["method"]),
                        str(row["seed"]),
                        row_backend(row),
                    )
                    seed_events = events_by_key.get(key, [])
                    per_seed_events.append(seed_events)
                    for event in seed_events:
                        elapsed = float_or_none(event.get("elapsed_seconds"))
                        if elapsed is not None and 0.0 <= elapsed <= x_max:
                            grid.add(log_runtime_value(elapsed, x_min))
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
                        linestyle=linestyle,
                        label=method if not plot_backends else "_nolegend_",
                    )

        ax_success.set_title(title)
        ax_success.set_ylabel("Successful solves (%)")
        ax_success.set_ylim(0.0, 100.0)
        ax_success.grid(True, alpha=0.3)
        add_success_legends(
            ax_success,
            methods=methods,
            colors=colors,
            backends=backends,
            plot_backends=plot_backends,
        )
        ax_makespan.set_xlabel("Runtime (s)")
        ax_makespan.set_ylabel("Median current best makespan")
        ax_success.set_xscale("log")
        ax_makespan.set_xscale("log")
        ax_makespan.set_xlim(x_min, x_max)
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
    result_csv_path: Path | None = None,
    event_csv_path: Path | None = None,
    result_rows: Sequence[dict[str, Any]],
    event_rows: Sequence[dict[str, Any]],
    plot_kind: str,
    plot_backends: bool = False,
) -> list[Path]:
    result_csv_path = result_csv_path or output_root / "results.csv"
    event_csv_path = event_csv_path or output_root / "solution_events.csv"
    result_rows = sorted(result_rows, key=result_sort_key)
    event_rows = sorted(event_rows, key=event_sort_key)
    write_csv(result_csv_path, RESULT_COLUMNS, result_rows)
    write_csv(event_csv_path, EVENT_COLUMNS, event_rows)
    if plot_kind == "success":
        return write_success_plots(
            result_rows,
            output_root,
            plot_backends=plot_backends,
        )
    if plot_kind == "anytime":
        return write_anytime_panel_plots(
            result_rows,
            event_rows,
            output_root,
            plot_backends=plot_backends,
        )
    return []
