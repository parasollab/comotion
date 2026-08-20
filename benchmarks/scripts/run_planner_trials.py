#!/usr/bin/env python3
"""Run paper-scale planner/backend trials with resumable per-trial records."""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Sequence

sys.dont_write_bytecode = True

from benchmark_runner_common import (
    DEFAULT_PARALLEL_ARC_CONFLICT_FIND_ASSIGNMENT,
    DEFAULT_BUILD_DIR,
    DEFAULT_RESULTS_DIR,
    PLANNER_LABELS,
    RUNTIME_CWD,
    csv_scalar,
    format_float,
    load_json,
    normalized_solution_events,
    slug,
    timestamp,
    write_csv,
)


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_PARAM_SPEC = SCRIPT_DIR.parent / "configs" / "planner_trial_params.json"
PINNED_EXEC_CODE = (
    "import os, sys; "
    "os.sched_setaffinity(0, {int(sys.argv[1])}); "
    "os.execv(sys.argv[2], sys.argv[2:])"
)

PAPER_METHODS = ("composite", "prioritized", "drrt", "stcbs", "arc")
KNOWN_METHODS = tuple(
    dict.fromkeys(
        (
            *PAPER_METHODS,
            "ao_arc",
            "parallel_arc",
            "composite_rrtstar",
            "composite_rrt_star",
            "composite_prmstar",
            "composite_prm_star",
            "composite_aorrtc",
            "cooperative_composite",
            "drrt_star",
            "ao_drrt",
        )
    )
)
DEFAULT_BACKENDS = ("vamp", "sphere", "fcl")

RESULT_COLUMNS = [
    "scenario",
    "case",
    "case_title",
    "num_robots",
    "primary_robot_count",
    "secondary_robot_count",
    "task_index",
    "seed",
    "method",
    "algorithm",
    "collision_backend",
    "time_limit_seconds",
    "success",
    "first_solution_time_seconds",
    "validation_time_seconds",
    "returncode",
    "timed_out",
    "trial_json",
    "metrics_json",
]

EVENT_COLUMNS = [
    "scenario",
    "case",
    "num_robots",
    "primary_robot_count",
    "secondary_robot_count",
    "task_index",
    "seed",
    "method",
    "algorithm",
    "collision_backend",
    "elapsed_seconds",
    "makespan_timesteps",
]


@dataclass(frozen=True)
class ScenarioDefinition:
    key: str
    aliases: tuple[str, ...]
    title: str
    executable: str
    base_args: tuple[str, ...]
    robot_counts: tuple[int, ...]
    default_num_trials: int
    default_time_limit: float
    parameter_profile: str
    task_indices: tuple[int, ...] = ()
    count_arg: str = "--num-robots"
    count_key_prefix: str = "n"
    count_label: str = "robots"
    secondary_count_arg: str | None = None
    secondary_count_multiplier: int = 0
    secondary_count_key_prefix: str = ""
    secondary_count_label: str = ""
    task_files: tuple[tuple[int, str], ...] = ()

    @property
    def task_based(self) -> bool:
        return bool(self.task_indices)

    def secondary_count(self, primary_count: int) -> int | None:
        if self.secondary_count_arg is None:
            return None
        return primary_count * self.secondary_count_multiplier

    def count_args(self, primary_count: int) -> tuple[str, ...]:
        args = [self.count_arg, str(primary_count)]
        secondary_count = self.secondary_count(primary_count)
        if self.secondary_count_arg is not None and secondary_count is not None:
            args.extend([self.secondary_count_arg, str(secondary_count)])
        return tuple(args)

    def count_key(self, primary_count: int) -> str:
        key = f"{self.count_key_prefix}{primary_count}"
        secondary_count = self.secondary_count(primary_count)
        if secondary_count is not None:
            key += f"_{self.secondary_count_key_prefix}{secondary_count}"
        return key

    def count_description(self, primary_count: int) -> str:
        description = f"{primary_count} {self.count_label}"
        secondary_count = self.secondary_count(primary_count)
        if secondary_count is not None:
            description += f" + {secondary_count} {self.secondary_count_label}"
        return description

    def total_count(self, primary_count: int) -> int:
        secondary_count = self.secondary_count(primary_count)
        if secondary_count is None:
            return primary_count
        return primary_count + secondary_count

    def task_file(self, primary_count: int) -> str | None:
        task_files = dict(self.task_files)
        return task_files.get(primary_count)


SCENARIOS: dict[str, ScenarioDefinition] = {
    "flying_spheres": ScenarioDefinition(
        key="flying_spheres",
        aliases=("rigid_body", "mobile_parallel", "mobile_parallel_2d"),
        title="Flying spheres, 2D mobile parallel",
        executable="mobile_robot_2d_crossing",
        base_args=("--scenario", "parallel"),
        robot_counts=(4, 8, 16, 32, 64, 128),
        default_num_trials=30,
        default_time_limit=10.0,
        parameter_profile="flying_spheres",
    ),
    "panda_cage": ScenarioDefinition(
        key="panda_cage",
        aliases=("manipulator", "panda"),
        title="Panda cage",
        executable="panda_cage",
        base_args=(),
        robot_counts=(4, 8, 16),
        default_num_trials=50,
        default_time_limit=150.0,
        parameter_profile="panda_cage",
        task_indices=(0, 1, 2, 3, 4),
    ),
    "heterogeneous_corridor": ScenarioDefinition(
        key="heterogeneous_corridor",
        aliases=("heterogeneous", "heterogenous", "panda_sphere_corridor"),
        title="Heterogeneous corridor",
        executable="heterogeneous_corridor",
        base_args=(),
        robot_counts=(4, 8, 16),
        default_num_trials=10,
        default_time_limit=150.0,
        parameter_profile="heterogeneous_corridor",
        task_indices=(0,),
        count_arg="--num-pandas",
        count_key_prefix="p",
        count_label="Pandas",
        secondary_count_arg="--num-spheres",
        secondary_count_multiplier=2,
        secondary_count_key_prefix="s",
        secondary_count_label="spheres",
        task_files=(
            (
                4,
                "resources/benchmarks/"
                "heterogeneous_four_pandas_eight_spheres_tasks.json",
            ),
            (
                8,
                "resources/benchmarks/"
                "heterogeneous_eight_pandas_sixteen_spheres_tasks.json",
            ),
            (
                16,
                "resources/benchmarks/"
                "heterogeneous_sixteen_pandas_thirtytwo_spheres_tasks.json",
            ),
        ),
    ),
}

SCENARIO_ALIASES = {
    alias: key
    for key, scenario in SCENARIOS.items()
    for alias in (key, *scenario.aliases)
}
UNSUPPORTED_SCENARIOS: dict[str, str] = {}


@dataclass(frozen=True)
class FlagSpec:
    flag: str
    kind: str = "value"
    false_flag: str | None = None


PARAMETER_FLAGS: dict[str, FlagSpec] = {
    "vamp_validation_strategy": FlagSpec("--vamp-validation-strategy"),
    "strrt_initial_batch_size": FlagSpec("--strrt-initial-batch-size"),
    "strrt_initial_time_factor": FlagSpec("--strrt-initial-time-factor"),
    "strrt_time_bound_factor_increase": FlagSpec(
        "--strrt-time-bound-factor-increase"
    ),
    "strrt_shuffle_priority_order": FlagSpec(
        "--strrt-shuffle-priority-order",
        "bool-pair",
        "--no-strrt-shuffle-priority-order",
    ),
    "strrt_return_first_solution": FlagSpec(
        "--strrt-return-first-solution", "bool-value"
    ),
    "strrt_rewiring": FlagSpec("--strrt-rewiring"),
    "drrt_roadmap_size": FlagSpec("--drrt-roadmap-size"),
    "drrt_iterations_per_batch": FlagSpec("--drrt-iterations-per-batch"),
    "drrt_cost_metric": FlagSpec("--drrt-cost-metric"),
    "drrt_tensor_search": FlagSpec("--drrt-tensor-search"),
    "drrt_local_connector": FlagSpec("--drrt-local-connector"),
    "drrt_exclude_roadmap_build_time": FlagSpec(
        "--drrt-exclude-roadmap-build-time", "true-flag"
    ),
    "composite_rrt_simplify_solution": FlagSpec(
        "--composite-rrt-simplify", "true-flag"
    ),
    "composite_rrt_use_makespan_metric": FlagSpec(
        "--composite-rrt-use-makespan-metric", "true-flag"
    ),
    "composite_aorrtc_max_internal_samples": FlagSpec(
        "--aorrtc-max-internal-samples"
    ),
    "composite_aorrtc_max_internal_vertices": FlagSpec(
        "--aorrtc-max-internal-vertices"
    ),
    "cooperative_rrt_worker_threads": FlagSpec("--cooperative-rrt-worker-threads"),
    "arc_initial_window": FlagSpec("--arc-initial-window"),
    "arc_expansion_step": FlagSpec("--arc-expansion-step"),
    "arc_expansion_policy": FlagSpec("--arc-expansion-policy"),
    "arc_expansion_multipliers": FlagSpec(
        "--arc-expansion-multipliers"
    ),
    "arc_initial_valid_expansion_policy": FlagSpec(
        "--arc-initial-valid-expansion-policy"
    ),
    "arc_initial_valid_expansion_step": FlagSpec(
        "--arc-initial-valid-expansion-step"
    ),
    "arc_initial_valid_expansion_multipliers": FlagSpec(
        "--arc-initial-valid-expansion-multipliers"
    ),
    "arc_initial_valid_expansion_symmetric": FlagSpec(
        "--arc-initial-valid-symmetric-expansion",
        "bool-pair",
        "--arc-initial-valid-asymmetric-expansion",
    ),
    "arc_cspace_bound_margin": FlagSpec("--arc-cspace-bound-margin"),
    "arc_min_cspace_bound_range": FlagSpec("--arc-min-cspace-bound-range"),
    "arc_simplification_max_shortcut_steps": FlagSpec(
        "--arc-simplification-max-shortcut-steps"
    ),
    "arc_simplification_max_empty_steps": FlagSpec(
        "--arc-simplification-max-empty-steps"
    ),
    "arc_simplification_max_smooth_steps": FlagSpec(
        "--arc-simplification-max-smooth-steps"
    ),
    "arc_simplification_max_passes": FlagSpec(
        "--arc-simplification-max-passes"
    ),
    "arc_conflict_simplification_max_shortcut_steps": FlagSpec(
        "--arc-conflict-simplification-max-shortcut-steps"
    ),
    "arc_conflict_simplification_max_empty_steps": FlagSpec(
        "--arc-conflict-simplification-max-empty-steps"
    ),
    "arc_conflict_simplification_max_smooth_steps": FlagSpec(
        "--arc-conflict-simplification-max-smooth-steps"
    ),
    "arc_conflict_simplification_max_passes": FlagSpec(
        "--arc-conflict-simplification-max-passes"
    ),
    "arc_local_composite_max_samples": FlagSpec(
        "--arc-local-composite-max-samples"
    ),
    "arc_local_composite_range": FlagSpec("--arc-local-composite-range"),
    "arc_local_composite_use_makespan_metric": FlagSpec(
        "--arc-local-composite-use-makespan-metric", "true-flag"
    ),
    "arc_simplify_initial_solutions": FlagSpec(
        "--arc-simplify-initial-solutions",
        "bool-pair",
        "--no-arc-simplify-initial-solutions",
    ),
    "arc_simplify_conflict_solutions": FlagSpec(
        "--arc-simplify-conflict-solutions",
        "bool-pair",
        "--no-arc-simplify-conflict-solutions",
    ),
    "arc_local_solvers": FlagSpec("--arc-local-solvers"),
    "arc_local_prioritized_max_iterations": FlagSpec(
        "--arc-local-prioritized-max-iterations"
    ),
    "ao_arc_local_bound_epsilon_timesteps": FlagSpec(
        "--ao-arc-local-bound-epsilon-timesteps"
    ),
    "or_parallel_worker_processes": FlagSpec("--or-parallel-worker-processes"),
    "parallel_arc_worker_processes": FlagSpec("--parallel-arc-worker-processes"),
    "parallel_arc_parallel_initial_plans": FlagSpec(
        "--parallel-arc-parallel-initial-plans",
        "bool-pair",
        "--no-parallel-arc-parallel-initial-plans",
    ),
    "parallel_arc_initial_solution_or": FlagSpec(
        "--parallel-arc-initial-solution-or",
        "bool-pair",
        "--no-parallel-arc-initial-solution-or",
    ),
    "parallel_arc_repair_duplicate_attempts": FlagSpec(
        "--parallel-arc-repair-duplicate-attempts",
        "bool-pair",
        "--no-parallel-arc-repair-duplicate-attempts",
    ),
    "parallel_arc_strategy": FlagSpec("--parallel-arc-strategy"),
    "parallel_arc_conflict_strategy": FlagSpec("--parallel-arc-conflict-strategy"),
    "parallel_arc_conflict_find_mode": FlagSpec("--parallel-arc-conflict-find-mode"),
    "parallel_arc_conflict_find_assignment": FlagSpec(
        "--parallel-arc-conflict-find-assignment"
    ),
    "parallel_arc_conflict_find_horizon": FlagSpec(
        "--parallel-arc-conflict-find-horizon"
    ),
    "stcbs_max_ct_nodes": FlagSpec("--stcbs-max-ct-nodes"),
    "stcbs_max_samples": FlagSpec("--stcbs-max-samples"),
    "stcbs_range": FlagSpec("--stcbs-range"),
}


@dataclass(frozen=True)
class TrialSpec:
    scenario: ScenarioDefinition
    num_robots: int
    task_index: int | None
    seed: int
    method: str
    backend: str
    time_limit: float
    resolution: int
    build_dir: Path
    output_root: Path
    params: dict[str, Any]
    params_args: tuple[str, ...]

    @property
    def case_key(self) -> str:
        return f"{self.scenario.key}_{self.scenario.count_key(self.num_robots)}"

    @property
    def case_title(self) -> str:
        count_description = self.scenario.count_description(self.num_robots)
        return f"{self.scenario.title}, {count_description}"

    @property
    def secondary_num_robots(self) -> int | None:
        return self.scenario.secondary_count(self.num_robots)

    @property
    def total_num_robots(self) -> int:
        return self.scenario.total_count(self.num_robots)

    @property
    def task_label(self) -> str:
        return "task_none" if self.task_index is None else f"task_{self.task_index:02d}"

    @property
    def method_label(self) -> str:
        return PLANNER_LABELS.get(self.method, self.method)

    @property
    def trial_id(self) -> str:
        return slug(
            f"{self.case_key}_{self.task_label}_{self.method}_{self.backend}_seed{self.seed}"
        )

    @property
    def trial_dir(self) -> Path:
        return (
            self.output_root
            / "trials"
            / self.scenario.key
            / self.scenario.count_key(self.num_robots)
            / self.task_label
            / self.method
            / self.backend
            / f"seed_{self.seed}"
        )

    @property
    def metrics_path(self) -> Path:
        return self.trial_dir / "metrics.json"

    @property
    def record_path(self) -> Path:
        return self.trial_dir / "trial.json"

    def command(self, metrics_path: Path | str = "<metrics-json>") -> list[str]:
        backend_args: list[str] = []
        if self.scenario.key == "panda_cage" and self.backend == "fcl":
            backend_args = ["--urdf", "panda/panda.urdf"]
        command = [
            str(self.build_dir / self.scenario.executable),
            *self.scenario.base_args,
            *self.scenario.count_args(self.num_robots),
            "--algorithm",
            self.method,
            "--collision-backend",
            self.backend,
            *backend_args,
            "--time-limit",
            format_float(self.time_limit),
            "--seed",
            str(self.seed),
            "--resolution",
            str(self.resolution),
            "--metrics-json",
            str(metrics_path),
            *self.params_args,
        ]
        if self.scenario.task_based:
            if self.task_index is None:
                raise RuntimeError(f"{self.scenario.key} requires a task index")
            task_file = self.scenario.task_file(self.num_robots)
            if task_file is not None:
                command.extend(["--task-file", task_file])
            command.extend(["--task-index", str(self.task_index)])
        return command

    def config_signature(self) -> str:
        payload = {
            "scenario": self.scenario.key,
            "num_robots": self.num_robots,
            "task_index": self.task_index,
            "seed": self.seed,
            "method": self.method,
            "backend": self.backend,
            "time_limit": self.time_limit,
            "resolution": self.resolution,
            "build_dir": str(self.build_dir),
            "command": self.command("<metrics-json>"),
            "params": self.params,
        }
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
        return hashlib.sha256(encoded).hexdigest()


def parse_csv_tokens(value: str) -> list[str]:
    return [token.strip() for token in value.split(",") if token.strip()]


def parse_int_csv(value: str) -> list[int]:
    return [int(token) for token in parse_csv_tokens(value)]


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json_file(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise RuntimeError(f"JSON file does not exist: {path}")
    try:
        with path.open() as handle:
            data = json.load(handle)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"{path} is not valid JSON") from exc
    if not isinstance(data, dict):
        raise RuntimeError(f"{path} must contain a JSON object")
    return data


def section_params(section: Any, context: str) -> dict[str, Any]:
    if section in (None, {}):
        return {}
    if not isinstance(section, dict):
        raise RuntimeError(f"{context} must be a JSON object")
    params = section.get("defaults", section)
    if not isinstance(params, dict):
        raise RuntimeError(f"{context} defaults must be a JSON object")
    return dict(params)


def merge_section(
    params: dict[str, Any],
    section: Any,
    context: str,
) -> None:
    params.update(section_params(section, context))


def planner_sections(container: Any, context: str) -> dict[str, dict[str, Any]]:
    if container in (None, {}):
        return {}
    if not isinstance(container, dict):
        raise RuntimeError(f"{context} must be a JSON object")

    out: dict[str, dict[str, Any]] = {}
    for key in ("methods", "planners"):
        raw_sections = container.get(key, {})
        if raw_sections in (None, {}):
            continue
        if not isinstance(raw_sections, dict):
            raise RuntimeError(f"{context}.{key} must be a JSON object")
        for planner, section in raw_sections.items():
            if not isinstance(planner, str):
                raise RuntimeError(f"{context}.{key} planner names must be strings")
            out.setdefault(planner, {}).update(
                section_params(section, f"{context}.{key}.{planner}")
            )
    return out


def planner_section(container: Any, method: str, context: str) -> dict[str, Any]:
    return planner_sections(container, context).get(method, {})


def resolve_planner_params(
    param_doc: dict[str, Any],
    *,
    scenario: ScenarioDefinition,
    num_robots: int,
    method: str,
) -> dict[str, Any]:
    params: dict[str, Any] = {}
    merge_section(params, param_doc.get("defaults", {}), "global defaults")

    scenarios = param_doc.get("scenarios", {})
    if not isinstance(scenarios, dict):
        raise RuntimeError("planner parameter spec 'scenarios' must be an object")
    try:
        scenario_doc = scenarios[scenario.parameter_profile]
    except KeyError as exc:
        available = ", ".join(sorted(scenarios))
        raise RuntimeError(
            f"No planner parameter profile for scenario '{scenario.parameter_profile}'. "
            f"Available: {available}"
        ) from exc
    if not isinstance(scenario_doc, dict):
        raise RuntimeError(f"Scenario profile {scenario.parameter_profile} must be an object")

    merge_section(
        params,
        scenario_doc.get("defaults", {}),
        f"scenario {scenario.parameter_profile} defaults",
    )
    robot_counts = scenario_doc.get("robot_counts", {})
    if robot_counts and not isinstance(robot_counts, dict):
        raise RuntimeError(
            f"scenario {scenario.parameter_profile} robot_counts must be an object"
        )
    robot_doc = robot_counts.get(str(num_robots), {}) if isinstance(robot_counts, dict) else {}
    merge_section(
        params,
        robot_doc,
        f"scenario {scenario.parameter_profile} n{num_robots} defaults",
    )

    merge_section(
        params,
        planner_section(param_doc, method, "global planner params"),
        f"global planner {method}",
    )
    merge_section(
        params,
        planner_section(
            scenario_doc,
            method,
            f"scenario {scenario.parameter_profile} planner params",
        ),
        f"scenario {scenario.parameter_profile} planner {method}",
    )
    merge_section(
        params,
        planner_section(
            robot_doc,
            method,
            f"scenario {scenario.parameter_profile} n{num_robots} planner params",
        ),
        f"scenario {scenario.parameter_profile} n{num_robots} planner {method}",
    )

    if method == "parallel_arc":
        params.setdefault(
            "parallel_arc_conflict_find_assignment",
            DEFAULT_PARALLEL_ARC_CONFLICT_FIND_ASSIGNMENT,
        )

    unknown = sorted(set(params) - set(PARAMETER_FLAGS))
    if unknown:
        available = ", ".join(sorted(PARAMETER_FLAGS))
        raise RuntimeError(
            f"Unknown planner parameter(s): {', '.join(unknown)}. "
            f"Available parameters: {available}"
        )
    return params


def bool_value(value: Any, key: str) -> bool:
    if isinstance(value, bool):
        return value
    raise RuntimeError(f"Planner parameter '{key}' must be boolean")


def param_value_text(value: Any) -> str:
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float):
        if not math.isfinite(value):
            raise RuntimeError("Planner parameter values must be finite")
        return format_float(value)
    return str(value)


def planner_params_to_args(params: dict[str, Any]) -> tuple[str, ...]:
    args: list[str] = []
    for key, spec in PARAMETER_FLAGS.items():
        if key not in params:
            continue
        value = params[key]
        if value is None:
            continue
        if spec.kind == "value":
            args.extend([spec.flag, param_value_text(value)])
        elif spec.kind == "bool-value":
            args.extend([spec.flag, "1" if bool_value(value, key) else "0"])
        elif spec.kind == "true-flag":
            if bool_value(value, key):
                args.append(spec.flag)
        elif spec.kind == "bool-pair":
            if bool_value(value, key):
                args.append(spec.flag)
            else:
                if spec.false_flag is None:
                    raise RuntimeError(f"Missing false flag for {key}")
                args.append(spec.false_flag)
        else:
            raise RuntimeError(f"Unknown flag spec kind '{spec.kind}' for {key}")
    return tuple(args)


def resolve_scenarios(value: str) -> list[ScenarioDefinition]:
    if value in {"default", "all"}:
        return [SCENARIOS[key] for key in SCENARIOS]
    scenarios: list[ScenarioDefinition] = []
    for token in parse_csv_tokens(value):
        if token in UNSUPPORTED_SCENARIOS:
            raise RuntimeError(UNSUPPORTED_SCENARIOS[token])
        try:
            key = SCENARIO_ALIASES[token]
        except KeyError as exc:
            available = ", ".join(sorted(SCENARIO_ALIASES))
            raise RuntimeError(
                f"Unknown scenario '{token}'. Available: {available}. "
                "Use heterogeneous_corridor or heterogeneous for the mixed Panda/sphere case."
            ) from exc
        scenario = SCENARIOS[key]
        if scenario not in scenarios:
            scenarios.append(scenario)
    if not scenarios:
        raise RuntimeError("At least one scenario is required")
    return scenarios


def resolve_methods(value: str) -> list[str]:
    if value in {"default", "all"}:
        return list(PAPER_METHODS)
    methods = parse_csv_tokens(value)
    if not methods:
        raise RuntimeError("At least one method is required")
    unknown = [method for method in methods if method not in KNOWN_METHODS]
    if unknown:
        available = ", ".join(KNOWN_METHODS)
        raise RuntimeError(
            f"Unknown method(s): {', '.join(unknown)}. Available: {available}"
        )
    return methods


def resolve_backends(value: str) -> list[str]:
    if value in {"default", "all"}:
        return list(DEFAULT_BACKENDS)
    backends = parse_csv_tokens(value)
    if not backends:
        raise RuntimeError("At least one collision backend is required")
    valid = set(DEFAULT_BACKENDS)
    unknown = [backend for backend in backends if backend not in valid]
    if unknown:
        raise RuntimeError(
            f"Unknown backend(s): {', '.join(unknown)}. Available: {', '.join(DEFAULT_BACKENDS)}"
        )
    return backends


def resolve_time_limits(value: str) -> dict[str, float]:
    limits: dict[str, float] = {}
    if not value:
        return limits
    for token in parse_csv_tokens(value):
        if "=" not in token:
            raise RuntimeError(
                "--time-limits entries must have the form scenario=seconds"
            )
        raw_key, raw_seconds = token.split("=", 1)
        key_token = raw_key.strip()
        if key_token in UNSUPPORTED_SCENARIOS:
            raise RuntimeError(UNSUPPORTED_SCENARIOS[key_token])
        try:
            key = SCENARIO_ALIASES[key_token]
        except KeyError as exc:
            available = ", ".join(sorted(SCENARIO_ALIASES))
            raise RuntimeError(
                f"Unknown scenario in --time-limits: {key_token}. Available: {available}"
            ) from exc
        seconds = float(raw_seconds)
        if seconds <= 0.0:
            raise RuntimeError("--time-limits values must be positive")
        limits[key] = seconds
    return limits


def task_indices_for_arg(value: str, scenario: ScenarioDefinition) -> tuple[int, ...]:
    if not scenario.task_based:
        return ()
    if value == "default":
        return scenario.task_indices
    indices = tuple(parse_int_csv(value))
    if not indices:
        raise RuntimeError("--task-indices must contain at least one task index")
    if any(index < 0 for index in indices):
        raise RuntimeError("--task-indices must be non-negative")
    return indices


def trial_task_seed_pairs(
    *,
    scenario: ScenarioDefinition,
    num_trials: int,
    task_indices: tuple[int, ...],
) -> list[tuple[int | None, int]]:
    if num_trials < 1:
        raise RuntimeError("Number of trials must be positive")
    if not scenario.task_based:
        return [(None, seed) for seed in range(num_trials)]

    pairs: list[tuple[int | None, int]] = []
    seed = 0
    while len(pairs) < num_trials:
        for task_index in task_indices:
            pairs.append((task_index, seed))
            if len(pairs) >= num_trials:
                break
        seed += 1
    return pairs


def default_num_trials_for(
    scenario: ScenarioDefinition,
    task_indices: tuple[int, ...],
    explicit_num_trials: int | None,
) -> int:
    if explicit_num_trials is not None:
        return explicit_num_trials
    if scenario.task_based and task_indices != scenario.task_indices:
        seeds_per_task = scenario.default_num_trials // len(scenario.task_indices)
        return len(task_indices) * seeds_per_task
    return scenario.default_num_trials


def build_trial_specs(args: argparse.Namespace) -> list[TrialSpec]:
    scenarios = resolve_scenarios(args.scenarios)
    methods = resolve_methods(args.methods)
    backends = resolve_backends(args.backends)
    requested_robot_counts = (
        None if args.robot_counts in {"default", "all"} else set(parse_int_csv(args.robot_counts))
    )
    time_limits = resolve_time_limits(args.time_limits)
    param_doc = read_json_file(args.planner_params)

    specs: list[TrialSpec] = []
    for scenario in scenarios:
        robot_counts = list(scenario.robot_counts)
        if requested_robot_counts is not None:
            robot_counts = [count for count in robot_counts if count in requested_robot_counts]
        if not robot_counts:
            requested = (
                args.robot_counts
                if requested_robot_counts is not None
                else ",".join(str(count) for count in scenario.robot_counts)
            )
            raise RuntimeError(
                f"No valid robot counts for scenario {scenario.key}: {requested}. "
                f"Supported: {', '.join(str(count) for count in scenario.robot_counts)}"
            )

        task_indices = task_indices_for_arg(args.task_indices, scenario)
        num_trials = default_num_trials_for(
            scenario,
            task_indices,
            args.num_trials,
        )
        time_limit = time_limits.get(
            scenario.key,
            args.time_limit if args.time_limit is not None else scenario.default_time_limit,
        )
        for num_robots in robot_counts:
            task_seed_pairs = trial_task_seed_pairs(
                scenario=scenario,
                num_trials=num_trials,
                task_indices=task_indices,
            )
            for task_index, seed in task_seed_pairs:
                for method in methods:
                    params = resolve_planner_params(
                        param_doc,
                        scenario=scenario,
                        num_robots=num_robots,
                        method=method,
                    )
                    params_args = planner_params_to_args(params)
                    for backend in backends:
                        specs.append(
                            TrialSpec(
                                scenario=scenario,
                                num_robots=num_robots,
                                task_index=task_index,
                                seed=seed,
                                method=method,
                                backend=backend,
                                time_limit=time_limit,
                                resolution=args.resolution,
                                build_dir=args.build_dir,
                                output_root=args.output_root,
                                params=params,
                                params_args=params_args,
                            )
                        )
    return specs


def first_solution_event(metrics: dict[str, Any]) -> dict[str, Any] | None:
    events = normalized_solution_events(metrics)
    return events[0] if events else None


def build_result_row(
    *,
    spec: TrialSpec,
    metrics: dict[str, Any],
    returncode: int | None,
    timed_out: bool,
) -> dict[str, Any]:
    first = first_solution_event(metrics)
    return {
        "scenario": spec.scenario.key,
        "case": spec.case_key,
        "case_title": spec.case_title,
        "num_robots": spec.total_num_robots,
        "primary_robot_count": spec.num_robots,
        "secondary_robot_count": csv_scalar(spec.secondary_num_robots),
        "task_index": "" if spec.task_index is None else spec.task_index,
        "seed": spec.seed,
        "method": spec.method_label,
        "algorithm": spec.method,
        "collision_backend": spec.backend,
        "time_limit_seconds": format_float(spec.time_limit),
        "success": bool(metrics.get("success")) and returncode == 0 and not timed_out,
        "first_solution_time_seconds": csv_scalar(
            first.get("elapsed_seconds") if first else None
        ),
        "validation_time_seconds": csv_scalar(
            metrics.get("validation_time_seconds")
        ),
        "returncode": "" if returncode is None else returncode,
        "timed_out": timed_out,
        "trial_json": str(spec.record_path),
        "metrics_json": str(spec.metrics_path),
    }


def build_event_rows(spec: TrialSpec, metrics: dict[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for event in normalized_solution_events(metrics):
        rows.append(
            {
                "scenario": spec.scenario.key,
                "case": spec.case_key,
                "num_robots": spec.total_num_robots,
                "primary_robot_count": spec.num_robots,
                "secondary_robot_count": csv_scalar(spec.secondary_num_robots),
                "task_index": "" if spec.task_index is None else spec.task_index,
                "seed": spec.seed,
                "method": spec.method_label,
                "algorithm": spec.method,
                "collision_backend": spec.backend,
                "elapsed_seconds": event["elapsed_seconds"],
                "makespan_timesteps": event["makespan_timesteps"],
            }
        )
    return rows


def row_sort_key(row: dict[str, Any]) -> tuple[Any, ...]:
    return (
        row.get("scenario", ""),
        int(row.get("num_robots", 0)),
        str(row.get("task_index", "")),
        int(row.get("seed", 0)),
        row.get("algorithm", ""),
        row.get("collision_backend", ""),
    )


def event_sort_key(row: dict[str, Any]) -> tuple[Any, ...]:
    return (
        *row_sort_key(row),
        float(row.get("elapsed_seconds", 0.0)),
    )


def safe_load_json(path: Path) -> dict[str, Any]:
    try:
        return load_json(path)
    except RuntimeError:
        return {}


def text_tail(value: str | bytes | None, limit: int = 4000) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        value = value.decode(errors="replace")
    return value[-limit:]


def available_worker_cores(limit: int) -> list[int]:
    try:
        cores = sorted(os.sched_getaffinity(0))
    except AttributeError:
        cores = list(range(os.cpu_count() or 1))
    if not cores:
        cores = [0]
    return cores[: min(limit, len(cores))]


def command_with_core_affinity(
    command: Sequence[str],
    pinned_core: int | None,
) -> tuple[list[str], str | None]:
    if pinned_core is None:
        return list(command), None

    taskset = shutil.which("taskset")
    if taskset:
        return [taskset, "-c", str(pinned_core), *command], "taskset"
    if hasattr(os, "sched_setaffinity"):
        return [
            sys.executable,
            "-c",
            PINNED_EXEC_CODE,
            str(pinned_core),
            *command,
        ], "python_sched_setaffinity"
    return list(command), None


def atomic_write_json(path: Path, doc: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    tmp_path.write_text(json.dumps(doc, indent=2, sort_keys=True) + "\n")
    tmp_path.replace(path)


def valid_trial_record(record: Any) -> bool:
    if not (
        isinstance(record, dict)
        and record.get("schema") == "comotion.planner_trial_record.v1"
        and record.get("status") == "complete"
        and isinstance(record.get("result_row"), dict)
        and isinstance(record.get("event_rows"), list)
    ):
        return False

    result_row = record["result_row"]
    event_rows = record["event_rows"]
    return all(column in result_row for column in RESULT_COLUMNS) and all(
        isinstance(event_row, dict)
        and all(column in event_row for column in EVENT_COLUMNS)
        for event_row in event_rows
    )


def reusable_trial_record(record: Any, spec: TrialSpec) -> bool:
    if not valid_trial_record(record):
        return False
    expected_signature = spec.config_signature()
    actual_signature = record.get("config_signature")
    if actual_signature != expected_signature:
        raise RuntimeError(
            "Existing completed trial record does not match the current run: "
            f"{spec.record_path}. Use --existing overwrite or choose a new "
            "--output-root."
        )
    return True


def run_one_trial(
    spec: TrialSpec,
    *,
    existing_policy: str,
    timeout_grace: float,
    index: int,
    total: int,
    pinned_core: int | None = None,
) -> dict[str, Any]:
    if existing_policy == "skip":
        existing = safe_load_json(spec.record_path)
        if reusable_trial_record(existing, spec):
            print(
                f"[{index}/{total}] skip {spec.trial_id}",
                flush=True,
            )
            return existing

    spec.trial_dir.mkdir(parents=True, exist_ok=True)
    if spec.metrics_path.exists():
        spec.metrics_path.unlink()

    command = spec.command(spec.metrics_path)
    launch_command, core_affinity_launcher = command_with_core_affinity(
        command,
        pinned_core,
    )
    timed_out = False
    returncode: int | None
    stdout = ""
    stderr = ""
    timeout_seconds = spec.time_limit + timeout_grace if timeout_grace >= 0.0 else None

    core_text = "" if core_affinity_launcher is None else f" core={pinned_core}"
    print(f"[{index}/{total}] run {spec.trial_id}{core_text}", flush=True)
    try:
        completed = subprocess.run(
            launch_command,
            cwd=RUNTIME_CWD,
            capture_output=True,
            text=True,
            timeout=timeout_seconds,
            check=False,
        )
        returncode = completed.returncode
        stdout = completed.stdout
        stderr = completed.stderr
    except subprocess.TimeoutExpired as exc:
        timed_out = True
        returncode = None
        stdout = text_tail(exc.stdout)
        stderr = text_tail(exc.stderr)

    metrics = safe_load_json(spec.metrics_path)
    result_row = build_result_row(
        spec=spec,
        metrics=metrics,
        returncode=returncode,
        timed_out=timed_out,
    )
    event_rows = build_event_rows(spec, metrics)
    record = {
        "schema": "comotion.planner_trial_record.v1",
        "status": "complete",
        "completed_utc": utc_now(),
        "trial_id": spec.trial_id,
        "config_signature": spec.config_signature(),
        "command": command,
        "launch_command": launch_command,
        "core_affinity": {
            "requested_core": pinned_core,
            "launcher": core_affinity_launcher,
            "pinned_core": pinned_core
            if core_affinity_launcher is not None
            else None,
        },
        "cwd": str(RUNTIME_CWD),
        "metrics_json": str(spec.metrics_path),
        "returncode": returncode,
        "timed_out": timed_out,
        "timeout_seconds": timeout_seconds,
        "stdout_tail": text_tail(stdout),
        "stderr_tail": text_tail(stderr),
        "result_row": result_row,
        "event_rows": event_rows,
    }
    atomic_write_json(spec.record_path, record)
    return record


def write_outputs(output_root: Path, records: Sequence[dict[str, Any]]) -> None:
    rows = [record["result_row"] for record in records]
    event_rows = [
        event_row
        for record in records
        for event_row in record.get("event_rows", [])
    ]
    write_csv(output_root / "results.csv", RESULT_COLUMNS, sorted(rows, key=row_sort_key))
    write_csv(
        output_root / "solution_events.csv",
        EVENT_COLUMNS,
        sorted(event_rows, key=event_sort_key),
    )


def aggregate_trial_record_valid(record: Any) -> bool:
    return (
        isinstance(record, dict)
        and record.get("schema") == "comotion.planner_trial_record.v1"
        and record.get("status") == "complete"
        and isinstance(record.get("result_row"), dict)
        and isinstance(record.get("event_rows"), list)
    )


def trial_record_key(record: dict[str, Any]) -> tuple[str, str] | None:
    trial_id = record.get("trial_id")
    if isinstance(trial_id, str) and trial_id:
        return ("trial_id", trial_id)

    row = record.get("result_row", {})
    if isinstance(row, dict):
        trial_json = row.get("trial_json")
        if isinstance(trial_json, str) and trial_json:
            return ("trial_json", trial_json)

    return None


def load_aggregate_trial_records(output_root: Path) -> list[dict[str, Any]]:
    trials_dir = output_root / "trials"
    if not trials_dir.is_dir():
        return []

    records: list[dict[str, Any]] = []
    for path in sorted(trials_dir.rglob("trial.json")):
        record = safe_load_json(path)
        if aggregate_trial_record_valid(record):
            records.append(record)
    return records


def merge_trial_records(
    existing_records: Sequence[dict[str, Any]],
    new_records: Sequence[dict[str, Any]],
) -> list[dict[str, Any]]:
    merged: dict[tuple[str, str], dict[str, Any]] = {}
    anonymous: list[dict[str, Any]] = []
    for record in (*existing_records, *new_records):
        if not aggregate_trial_record_valid(record):
            continue
        key = trial_record_key(record)
        if key is None:
            anonymous.append(record)
        else:
            merged[key] = record
    return [*merged.values(), *anonymous]


def pruning_event_key(event: dict[str, Any]) -> tuple[Any, ...]:
    return (
        event.get("scenario"),
        event.get("primary_robot_count"),
        event.get("secondary_robot_count"),
        event.get("task_index"),
        event.get("method"),
        event.get("backend"),
    )


def load_pruning_events(output_root: Path) -> list[dict[str, Any]]:
    doc = safe_load_json(output_root / "pruned_combinations.json")
    events = doc.get("events")
    if not isinstance(events, list):
        return []
    return [event for event in events if isinstance(event, dict)]


def merge_pruning_events(
    existing_events: Sequence[dict[str, Any]],
    new_events: Sequence[dict[str, Any]],
) -> list[dict[str, Any]]:
    merged: dict[tuple[Any, ...], dict[str, Any]] = {}
    for event in (*existing_events, *new_events):
        merged[pruning_event_key(event)] = dict(event)
    return list(merged.values())


def write_pruning_report(
    output_root: Path,
    events: Sequence[dict[str, Any]],
) -> Path:
    path = output_root / "pruned_combinations.json"
    atomic_write_json(
        path,
        {
            "schema": "comotion.planner_trials_pruning.v1",
            "created_utc": utc_now(),
            "rule": (
                "Within each scenario and task index, method/backend "
                "combinations with zero successful trials at a team size are "
                "skipped for larger team sizes. The task index is used as a "
                "cross-size pruning key for task-based scenarios."
            ),
            "events": list(events),
        },
    )
    return path


def trial_stages(
    specs: Sequence[TrialSpec],
) -> list[tuple[tuple[str, int], list[TrialSpec]]]:
    stages: dict[tuple[str, int], list[TrialSpec]] = {}
    scenario_order: dict[str, int] = {}
    for spec in specs:
        scenario_order.setdefault(spec.scenario.key, len(scenario_order))
        stages.setdefault((spec.scenario.key, spec.num_robots), []).append(spec)
    return sorted(
        stages.items(),
        key=lambda item: (scenario_order[item[0][0]], item[0][1]),
    )


def robot_counts_by_scenario(specs: Sequence[TrialSpec]) -> dict[str, list[int]]:
    counts: dict[str, set[int]] = {}
    for spec in specs:
        counts.setdefault(spec.scenario.key, set()).add(spec.num_robots)
    return {
        scenario_key: sorted(scenario_counts)
        for scenario_key, scenario_counts in counts.items()
    }


def total_robot_counts_by_scenario(specs: Sequence[TrialSpec]) -> dict[str, list[int]]:
    counts: dict[str, set[int]] = {}
    for spec in specs:
        counts.setdefault(spec.scenario.key, set()).add(spec.total_num_robots)
    return {
        scenario_key: sorted(scenario_counts)
        for scenario_key, scenario_counts in counts.items()
    }


ProblemMethodBackendKey = tuple[int | None, str, str]
BackendCascadeKey = tuple[str, int, int | None, int, str]


def problem_method_backend_key(spec: TrialSpec) -> ProblemMethodBackendKey:
    return (spec.task_index, spec.method, spec.backend)


def record_problem_method_backend(
    record: dict[str, Any],
) -> ProblemMethodBackendKey | None:
    row = record.get("result_row", {})
    if not isinstance(row, dict):
        return None
    task_index_value = row.get("task_index")
    if task_index_value in (None, ""):
        task_index = None
    elif isinstance(task_index_value, int) and not isinstance(
        task_index_value, bool
    ):
        task_index = task_index_value
    else:
        try:
            task_index = int(task_index_value)
        except (TypeError, ValueError):
            return None
    method = row.get("algorithm")
    backend = row.get("collision_backend")
    if not isinstance(method, str) or not isinstance(backend, str):
        return None
    return (task_index, method, backend)


def record_succeeded(record: dict[str, Any]) -> bool:
    row = record.get("result_row", {})
    return isinstance(row, dict) and bool(row.get("success"))


def backend_cascade_key(spec: TrialSpec) -> BackendCascadeKey:
    return (
        spec.scenario.key,
        spec.num_robots,
        spec.task_index,
        spec.seed,
        spec.method,
    )


def backend_cascade_groups(
    specs: Sequence[TrialSpec],
) -> list[list[TrialSpec]]:
    groups: dict[BackendCascadeKey, list[TrialSpec]] = {}
    for spec in specs:
        groups.setdefault(backend_cascade_key(spec), []).append(spec)
    priority = {backend: index for index, backend in enumerate(DEFAULT_BACKENDS)}
    return [
        sorted(group, key=lambda spec: priority[spec.backend])
        for group in groups.values()
    ]


def record_exhausted_planning_budget(record: dict[str, Any]) -> bool:
    if record_succeeded(record):
        return False
    if bool(record.get("timed_out")):
        return True
    # The benchmark applications return code zero when the planner reports
    # TIMEOUT/no solution. Nonzero exits are execution errors and must not
    # silently suppress the remaining validation backends.
    return record.get("returncode") == 0


def run_backend_cascade_group(
    specs: Sequence[TrialSpec],
    *,
    args: argparse.Namespace,
    indices: dict[str, int],
    total: int,
    pinned_core: int,
) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    blocking_backend: str | None = None
    for spec in specs:
        index = indices[spec.trial_id]
        if blocking_backend is not None:
            print(
                f"[{index}/{total}] skip_cascade {spec.trial_id} "
                f"after {blocking_backend} timeout",
                flush=True,
            )
            continue
        record = run_one_trial(
            spec,
            existing_policy=args.existing,
            timeout_grace=args.timeout_grace,
            index=index,
            total=total,
            pinned_core=pinned_core,
        )
        records.append(record)
        if record_exhausted_planning_budget(record):
            blocking_backend = spec.backend
    return records


def run_trial_batch(
    specs: Sequence[TrialSpec],
    *,
    args: argparse.Namespace,
    first_index: int,
    total: int,
) -> list[dict[str, Any]]:
    if not specs:
        return []

    worker_cores = available_worker_cores(args.cores)
    if len(worker_cores) < args.cores:
        print(
            f"requested_cores={args.cores} available_pinned_cores={len(worker_cores)}",
            flush=True,
        )

    groups = backend_cascade_groups(specs)
    indices = {
        spec.trial_id: first_index + offset for offset, spec in enumerate(specs)
    }

    if len(worker_cores) == 1:
        records: list[dict[str, Any]] = []
        for group in groups:
            records.extend(
                run_backend_cascade_group(
                    group,
                    args=args,
                    indices=indices,
                    total=total,
                    pinned_core=worker_cores[0],
                )
            )
        return records

    records = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=len(worker_cores)) as pool:
        futures = {
            pool.submit(
                run_backend_cascade_group,
                group,
                args=args,
                indices=indices,
                total=total,
                pinned_core=worker_cores[offset % len(worker_cores)],
            ): group
            for offset, group in enumerate(groups)
        }
        for future in concurrent.futures.as_completed(futures):
            records.append(future.result())
    return records


def prune_after_stage(
    *,
    stage_specs: Sequence[TrialSpec],
    stage_records: Sequence[dict[str, Any]],
    counts_by_scenario: dict[str, list[int]],
    pruned_combinations: dict[str, set[ProblemMethodBackendKey]],
) -> list[dict[str, Any]]:
    if not stage_specs:
        return []

    scenario_key = stage_specs[0].scenario.key
    num_robots = stage_specs[0].num_robots
    larger_counts = [
        count for count in counts_by_scenario.get(scenario_key, []) if count > num_robots
    ]
    if not larger_counts:
        return []

    trial_counts: dict[ProblemMethodBackendKey, int] = {}
    example_specs: dict[ProblemMethodBackendKey, TrialSpec] = {}
    for spec in stage_specs:
        combo = problem_method_backend_key(spec)
        trial_counts[combo] = trial_counts.get(combo, 0) + 1
        example_specs.setdefault(combo, spec)

    success_counts: dict[ProblemMethodBackendKey, int] = {}
    for record in stage_records:
        combo = record_problem_method_backend(record)
        if combo is not None and record_succeeded(record):
            success_counts[combo] = success_counts.get(combo, 0) + 1

    events: list[dict[str, Any]] = []
    scenario_pruned = pruned_combinations.setdefault(scenario_key, set())
    for combo in sorted(
        trial_counts,
        key=lambda value: (
            -1 if value[0] is None else value[0],
            value[1],
            value[2],
        ),
    ):
        if success_counts.get(combo, 0) != 0 or combo in scenario_pruned:
            continue
        scenario_pruned.add(combo)
        spec = example_specs[combo]
        event = {
            "scenario": scenario_key,
            "case": spec.case_key,
            "num_robots": spec.total_num_robots,
            "primary_robot_count": spec.num_robots,
            "secondary_robot_count": spec.secondary_num_robots,
            "task_index": spec.task_index,
            "task_label": spec.task_label,
            "method": spec.method,
            "backend": spec.backend,
            "trial_count": trial_counts[combo],
            "success_count": 0,
            "pruned_larger_robot_counts": [
                spec.scenario.total_count(count) for count in larger_counts
            ],
            "pruned_larger_primary_robot_counts": larger_counts,
            "pruned_larger_secondary_robot_counts": [
                spec.scenario.secondary_count(count) for count in larger_counts
            ],
        }
        events.append(event)
        print(
            "prune "
            f"{scenario_key} n={num_robots} {spec.task_label} "
            f"{spec.method}/{spec.backend} "
            f"after 0/{trial_counts[combo]} successes; "
            f"larger_counts={','.join(str(count) for count in larger_counts)}",
            flush=True,
        )
    return events


def run_trials_with_pruning(
    specs: Sequence[TrialSpec],
    *,
    args: argparse.Namespace,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    records: list[dict[str, Any]] = []
    pruning_events: list[dict[str, Any]] = []
    pruned_combinations: dict[str, set[ProblemMethodBackendKey]] = {}
    counts_by_scenario = robot_counts_by_scenario(specs)
    next_index = 1

    for (scenario_key, num_robots), stage_specs in trial_stages(specs):
        scenario_pruned = pruned_combinations.setdefault(scenario_key, set())
        active_specs = [
            spec
            for spec in stage_specs
            if problem_method_backend_key(spec) not in scenario_pruned
        ]
        skipped = len(stage_specs) - len(active_specs)
        if skipped:
            print(
                f"skip_pruned scenario={scenario_key} n={num_robots} trials={skipped}",
                flush=True,
            )
        if not active_specs:
            continue

        print(
            f"stage scenario={scenario_key} n={num_robots} trials={len(active_specs)}",
            flush=True,
        )
        stage_records = run_trial_batch(
            active_specs,
            args=args,
            first_index=next_index,
            total=len(specs),
        )
        next_index += len(active_specs)
        records.extend(stage_records)
        if getattr(args, "cross_size_pruning", True):
            pruning_events.extend(
                prune_after_stage(
                    stage_specs=active_specs,
                    stage_records=stage_records,
                    counts_by_scenario=counts_by_scenario,
                    pruned_combinations=pruned_combinations,
                )
            )

    return records, pruning_events


def write_manifest(
    output_root: Path,
    *,
    args: argparse.Namespace,
    specs: Sequence[TrialSpec],
    selected_scenarios: Sequence[ScenarioDefinition],
    methods: Sequence[str],
    backends: Sequence[str],
) -> None:
    output_root.mkdir(parents=True, exist_ok=True)
    selected_primary_counts = robot_counts_by_scenario(specs)
    selected_total_counts = total_robot_counts_by_scenario(specs)
    manifest = {
        "schema": "comotion.planner_trials_manifest.v1",
        "created_utc": utc_now(),
        "command_line": sys.argv,
        "build_dir": str(args.build_dir),
        "runtime_cwd": str(RUNTIME_CWD),
        "output_root": str(output_root),
        "planner_params": {
            "path": str(args.planner_params),
            "sha256": file_sha256(args.planner_params),
        },
        "existing_policy": args.existing,
        "cores": args.cores,
        "pinned_cores": available_worker_cores(args.cores),
        "resolution": args.resolution,
        "timeout_grace_seconds": args.timeout_grace,
        "trial_count": len(specs),
        "trial_count_before_pruning": len(specs),
        "pruning_enabled": getattr(args, "cross_size_pruning", True),
        "backend_cascade_enabled": True,
        "backend_cascade_order": list(DEFAULT_BACKENDS),
        "backend_cascade_rule": (
            "Within one scenario/team-size/task/seed/method configuration, "
            "run VAMP then sphere then FCL. A planner timeout/no-solution "
            "skips only the remaining backends for that exact configuration."
        ),
        "pruning_rule": (
            "Within each scenario and task index, method/backend "
            "combinations with zero successful trials at a team size are skipped "
            "for larger team sizes. The task index is used as a cross-size "
            "pruning key for task-based scenarios."
        ),
        "methods": list(methods),
        "backends": list(backends),
        "scenarios": [
            {
                "key": scenario.key,
                "title": scenario.title,
                "executable": scenario.executable,
                "base_args": list(scenario.base_args),
                "primary_robot_counts": selected_primary_counts.get(
                    scenario.key, []
                ),
                "total_robot_counts": selected_total_counts.get(scenario.key, []),
                "case_keys": [
                    scenario.count_key(count)
                    for count in selected_primary_counts.get(scenario.key, [])
                ],
                "count_arg": scenario.count_arg,
                "secondary_count_arg": scenario.secondary_count_arg,
                "secondary_count_multiplier": scenario.secondary_count_multiplier,
                "default_num_trials": scenario.default_num_trials,
                "default_time_limit_seconds": scenario.default_time_limit,
                "task_indices": list(scenario.task_indices),
                "task_files": {
                    str(count): {
                        "path": path,
                        "sha256": file_sha256(RUNTIME_CWD / path),
                    }
                    for count, path in scenario.task_files
                },
            }
            for scenario in selected_scenarios
        ],
        "suite_defaults": {
            "flying_spheres": {
                "primary_robot_counts": [4, 8, 16, 32, 64, 128],
                "total_robot_counts": [4, 8, 16, 32, 64, 128],
                "trials_per_method_backend_robot_count": 30,
                "time_limit_seconds": 10.0,
            },
            "panda_cage": {
                "primary_robot_counts": [4, 8, 16],
                "total_robot_counts": [4, 8, 16],
                "tasks": 5,
                "seeds_per_task": 10,
                "trials_per_method_backend_robot_count": 50,
                "time_limit_seconds": 150.0,
            },
            "heterogeneous_corridor": {
                "team_sizes": ["p4_s8", "p8_s16", "p16_s32"],
                "primary_robot_counts": [4, 8, 16],
                "total_robot_counts": [12, 24, 48],
                "tasks": 1,
                "trials_per_method_backend_robot_count": 10,
                "time_limit_seconds": 150.0,
            },
        },
    }
    atomic_write_json(output_root / "manifest.json", manifest)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run the paper-scale planner trials across VAMP, FCL, and sphere "
            "collision backends. Defaults cover Panda cage, flying spheres, "
            "and heterogeneous Panda/sphere corridor trials."
        )
    )
    parser.add_argument(
        "--scenarios",
        default="default",
        help=(
            "Comma-separated scenarios, or 'default'/'all'. "
            "Available: flying_spheres, panda_cage, heterogeneous_corridor."
        ),
    )
    parser.add_argument(
        "--methods",
        default="default",
        help=(
            "Comma-separated planner algorithms, or 'default'/'all'. "
            "Default paper set: composite,prioritized,drrt,stcbs,arc."
        ),
    )
    parser.add_argument(
        "--robot-counts",
        default="default",
        help="Comma-separated robot counts to keep, or 'default'/'all'.",
    )
    parser.add_argument(
        "--backends",
        default="default",
        help="Comma-separated collision backends, or 'default'/'all'. Default: vamp,fcl,sphere.",
    )
    parser.add_argument(
        "--num-trials",
        type=int,
        help=(
            "Trials per method/backend/team-size. Default suite: "
            "30 for flying_spheres; 50 for panda_cage; "
            "10 for heterogeneous_corridor."
        ),
    )
    parser.add_argument(
        "--task-indices",
        default="default",
        help=(
            "Task indices for task-based scenarios. Default Panda cage tasks are 0..4."
        ),
    )
    parser.add_argument(
        "--time-limit",
        type=float,
        help="Global per-trial time limit in seconds.",
    )
    parser.add_argument(
        "--time-limits",
        default="",
        help=(
            "Scenario-specific limits, e.g. "
            "flying_spheres=100,panda_cage=1000,heterogeneous_corridor=1000. "
            "Overrides --time-limit for named scenarios."
        ),
    )
    parser.add_argument("--cores", type=int, default=1)
    parser.add_argument(
        "--cross-size-pruning",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Prune zero-success task/method/backend combinations at larger "
            "team sizes (default: enabled)."
        ),
    )
    parser.add_argument("--resolution", type=int, default=128)
    parser.add_argument("--build-dir", type=Path, default=DEFAULT_BUILD_DIR)
    parser.add_argument(
        "--planner-params",
        type=Path,
        default=DEFAULT_PARAM_SPEC,
        help="Planner-parameter JSON file.",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=None,
        help="Output directory. Default: benchmarks/results/planner_trials_<UTC timestamp>.",
    )
    parser.add_argument(
        "--existing",
        choices=("skip", "overwrite"),
        default="skip",
        help="How to handle completed trial records already present in the output directory.",
    )
    parser.add_argument(
        "--timeout-grace",
        type=float,
        default=30.0,
        help="Seconds added to each planner time limit for the process timeout.",
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--dry-run-limit",
        type=int,
        default=50,
        help="Maximum commands to print in --dry-run mode; use 0 to print all.",
    )
    args = parser.parse_args()

    if args.cores < 1:
        raise RuntimeError("--cores must be at least 1")
    if args.resolution < 1:
        raise RuntimeError("--resolution must be at least 1")
    if args.num_trials is not None and args.num_trials < 1:
        raise RuntimeError("--num-trials must be at least 1")
    if args.time_limit is not None and args.time_limit <= 0.0:
        raise RuntimeError("--time-limit must be positive")
    if args.timeout_grace < 0.0:
        raise RuntimeError("--timeout-grace must be non-negative")
    if args.dry_run_limit < 0:
        raise RuntimeError("--dry-run-limit must be non-negative")
    args.output_root = args.output_root or DEFAULT_RESULTS_DIR / f"planner_trials_{timestamp()}"
    return args


def main() -> int:
    try:
        args = parse_args()
        selected_scenarios = resolve_scenarios(args.scenarios)
        methods = resolve_methods(args.methods)
        backends = resolve_backends(args.backends)
        specs = build_trial_specs(args)

        if args.dry_run:
            limit = len(specs) if args.dry_run_limit == 0 else args.dry_run_limit
            for spec in specs[:limit]:
                print(" ".join(spec.command(spec.metrics_path)))
            if limit < len(specs):
                print(f"omitted_commands: {len(specs) - limit}")
            for _, stage_specs in trial_stages(specs):
                example = stage_specs[0]
                print(
                    f"planned_case: {example.case_key} trials={len(stage_specs)}"
                )
            print(f"planned_trials: {len(specs)}")
            print(f"planned_trials_before_pruning: {len(specs)}")
            print(
                "pruning_enabled: "
                + str(args.cross_size_pruning).lower()
            )
            print("backend_cascade_enabled: true")
            print(
                "backend_cascade_order: " + ",".join(DEFAULT_BACKENDS)
            )
            print(
                "pinned_cores: "
                + ",".join(str(core) for core in available_worker_cores(args.cores))
            )
            print(
                "pruning_note: dry-run shows the unpruned command set; "
                "zero-success task/method/backend combinations are skipped for "
                "larger team sizes after each completed scenario/team-size "
                "stage, using task index as the cross-size key for task-based "
                "scenarios."
            )
            print(f"output_root: {args.output_root}")
            print(f"planner_params: {args.planner_params}")
            return 0

        existing_records = load_aggregate_trial_records(args.output_root)
        selected_trial_keys = {
            ("trial_id", spec.trial_id) for spec in specs
        }
        preserve_manifest = (
            (args.output_root / "manifest.json").is_file()
            and any(
                trial_record_key(record) not in selected_trial_keys
                for record in existing_records
            )
        )
        if preserve_manifest:
            print(f"preserve_manifest: {args.output_root / 'manifest.json'}")
        else:
            write_manifest(
                args.output_root,
                args=args,
                specs=specs,
                selected_scenarios=selected_scenarios,
                methods=methods,
                backends=backends,
            )

        records, pruning_events = run_trials_with_pruning(specs, args=args)

        aggregate_records = merge_trial_records(
            load_aggregate_trial_records(args.output_root),
            records,
        )
        write_outputs(args.output_root, aggregate_records)
        pruning_path = write_pruning_report(
            args.output_root,
            merge_pruning_events(
                load_pruning_events(args.output_root),
                pruning_events,
            ),
        )
        print(f"manifest: {args.output_root / 'manifest.json'}")
        print(f"results_csv: {args.output_root / 'results.csv'}")
        print(f"solution_events_csv: {args.output_root / 'solution_events.csv'}")
        print(f"pruned_combinations_json: {pruning_path}")
        print(f"trial_records: {args.output_root / 'trials'}")
        print(f"aggregate_trial_records: {len(aggregate_records)}")
        return 0
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
