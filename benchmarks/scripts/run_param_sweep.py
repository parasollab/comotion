#!/usr/bin/env python3
"""Run JSON-configured planner parameter sweeps."""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
import hashlib
import itertools
import json
import math
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
    format_float,
    load_json,
    parse_csv_tokens,
    slug,
    timestamp,
)
from run_planner_trials import planner_params_to_args


RESULT_COLUMNS = [
    "app",
    "app_title",
    "method",
    "method_label",
    "param_set",
    "seed",
    "success",
    "planning_time_seconds",
    "effective_planning_time_seconds",
    "returncode",
    "timed_out",
    "trial_json",
    "metrics_json",
]

SUMMARY_COLUMNS = [
    "app",
    "method",
    "param_set",
    "trial_count",
    "success_count",
    "mean_successful_planning_time_seconds",
    "mean_effective_planning_time_seconds",
    "best",
]


@dataclass(frozen=True)
class AppSpec:
    key: str
    title: str
    executable: str
    args: tuple[str, ...]
    time_limit: float
    methods: tuple[str, ...]
    collision_backend: str
    resolution: int
    trials_per_param_set: int
    seeds: tuple[int, ...] | None
    base_params: dict[str, Any]
    method_defaults: dict[str, dict[str, Any]]
    param_grid: dict[str, Any]


@dataclass(frozen=True)
class ParamSet:
    name: str
    params: dict[str, Any]
    extra_args: tuple[str, ...] = ()


@dataclass(frozen=True)
class TrialSpec:
    app: AppSpec
    method: str
    param_set: ParamSet
    seed: int
    build_dir: Path
    output_root: Path
    timeout_grace: float

    @property
    def method_label(self) -> str:
        return PLANNER_LABELS.get(self.method, self.method)

    @property
    def trial_id(self) -> str:
        return slug(f"{self.app.key}_{self.method}_{self.param_set.name}_seed{self.seed}")

    @property
    def trial_dir(self) -> Path:
        return (
            self.output_root
            / "trials"
            / self.app.key
            / self.method
            / self.param_set.name
            / f"seed_{self.seed}"
        )

    @property
    def metrics_path(self) -> Path:
        return self.trial_dir / "metrics.json"

    @property
    def record_path(self) -> Path:
        return self.trial_dir / "trial.json"

    def command(self, metrics_path: Path | str = "<metrics-json>") -> list[str]:
        return [
            str(self.build_dir / self.app.executable),
            *self.app.args,
            "--algorithm",
            self.method,
            "--collision-backend",
            self.app.collision_backend,
            "--time-limit",
            format_float(self.app.time_limit),
            "--seed",
            str(self.seed),
            "--resolution",
            str(self.app.resolution),
            "--metrics-json",
            str(metrics_path),
            *planner_params_to_args(self.param_set.params),
            *self.param_set.extra_args,
        ]

    def config_signature(self) -> str:
        payload = {
            "app": self.app.key,
            "executable": self.app.executable,
            "args": self.app.args,
            "method": self.method,
            "param_set": self.param_set.name,
            "params": self.param_set.params,
            "extra_args": self.param_set.extra_args,
            "seed": self.seed,
            "collision_backend": self.app.collision_backend,
            "time_limit": self.app.time_limit,
            "resolution": self.app.resolution,
            "build_dir": str(self.build_dir),
        }
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
        return hashlib.sha256(encoded).hexdigest()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def finite_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def read_json_object(path: Path) -> dict[str, Any]:
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


def atomic_write_json(path: Path, doc: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    tmp_path.write_text(json.dumps(doc, indent=2, sort_keys=True) + "\n")
    tmp_path.replace(path)


def write_csv(path: Path, columns: Sequence[str], rows: Iterable[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(columns))
        writer.writeheader()
        for row in rows:
            writer.writerow({column: row.get(column, "") for column in columns})


def merge_dicts(*values: dict[str, Any]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for value in values:
        out.update(value)
    return out


def as_string_tuple(value: Any, context: str) -> tuple[str, ...]:
    if value is None:
        return ()
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise RuntimeError(f"{context} must be a list of strings")
    return tuple(value)


def as_param_dict(value: Any, context: str) -> dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise RuntimeError(f"{context} must be an object")
    return dict(value)


def parse_seeds(value: Any, trials_per_param_set: int, context: str) -> tuple[int, ...]:
    if value is None:
        return tuple(range(trials_per_param_set))
    if not isinstance(value, list) or not all(isinstance(item, int) for item in value):
        raise RuntimeError(f"{context} seeds must be a list of integers")
    if not value:
        raise RuntimeError(f"{context} seeds must not be empty")
    return tuple(value)


def parse_app_specs(config: dict[str, Any]) -> dict[str, AppSpec]:
    defaults = as_param_dict(config.get("defaults", {}), "defaults")
    default_backend = str(defaults.get("collision_backend", "vamp"))
    default_resolution = int(defaults.get("resolution", 128))
    default_trials = int(defaults.get("trials_per_param_set", 1))
    default_base_params = as_param_dict(defaults.get("base_params", {}), "defaults.base_params")
    default_methods = tuple(defaults.get("methods", ()))
    if default_methods and not all(isinstance(method, str) for method in default_methods):
        raise RuntimeError("defaults.methods must be a list of strings")

    apps = config.get("apps")
    if not isinstance(apps, dict) or not apps:
        raise RuntimeError("config must contain a non-empty 'apps' object")

    out: dict[str, AppSpec] = {}
    for key, raw_app in apps.items():
        if not isinstance(raw_app, dict):
            raise RuntimeError(f"apps.{key} must be an object")
        trials = int(raw_app.get("trials_per_param_set", default_trials))
        if trials < 1:
            raise RuntimeError(f"apps.{key}.trials_per_param_set must be at least 1")
        methods_raw = raw_app.get("methods", default_methods)
        if not isinstance(methods_raw, (list, tuple)) or not all(
            isinstance(method, str) for method in methods_raw
        ):
            raise RuntimeError(f"apps.{key}.methods must be a list of strings")
        if not methods_raw:
            raise RuntimeError(f"apps.{key}.methods must not be empty")

        time_limit = float(raw_app.get("time_limit"))
        if time_limit <= 0.0:
            raise RuntimeError(f"apps.{key}.time_limit must be positive")

        app_base_params = merge_dicts(
            default_base_params,
            as_param_dict(raw_app.get("base_params", {}), f"apps.{key}.base_params"),
        )
        method_defaults = raw_app.get("method_defaults", {})
        if not isinstance(method_defaults, dict):
            raise RuntimeError(f"apps.{key}.method_defaults must be an object")
        parsed_method_defaults = {
            str(method): as_param_dict(params, f"apps.{key}.method_defaults.{method}")
            for method, params in method_defaults.items()
        }
        param_grid = raw_app.get("param_grid", {})
        if not isinstance(param_grid, dict):
            raise RuntimeError(f"apps.{key}.param_grid must be an object")

        out[str(key)] = AppSpec(
            key=str(key),
            title=str(raw_app.get("title", key)),
            executable=str(raw_app["executable"]),
            args=as_string_tuple(raw_app.get("args", []), f"apps.{key}.args"),
            time_limit=time_limit,
            methods=tuple(methods_raw),
            collision_backend=str(raw_app.get("collision_backend", default_backend)),
            resolution=int(raw_app.get("resolution", default_resolution)),
            trials_per_param_set=trials,
            seeds=parse_seeds(raw_app.get("seeds"), trials, f"apps.{key}"),
            base_params=app_base_params,
            method_defaults=parsed_method_defaults,
            param_grid=param_grid,
        )
    return out


def expand_grid_object(grid: dict[str, Any]) -> list[ParamSet]:
    keys = list(grid.keys())
    values: list[list[Any]] = []
    for key in keys:
        raw = grid[key]
        if not isinstance(raw, list) or not raw:
            raise RuntimeError(f"grid parameter '{key}' must be a non-empty list")
        values.append(raw)
    param_sets: list[ParamSet] = []
    for combo in itertools.product(*values):
        params = dict(zip(keys, combo))
        name = "_".join(f"{slug(key)}_{slug(value)}" for key, value in params.items())
        param_sets.append(ParamSet(name=name, params=params))
    return param_sets


def parse_param_sets(raw_grid: Any, method: str) -> list[ParamSet]:
    if raw_grid is None:
        return [ParamSet(name="default", params={})]
    if isinstance(raw_grid, list):
        param_sets: list[ParamSet] = []
        for index, item in enumerate(raw_grid):
            if not isinstance(item, dict):
                raise RuntimeError(f"param_grid.{method}[{index}] must be an object")
            params = as_param_dict(item.get("params", item), f"param_grid.{method}[{index}].params")
            name = str(item.get("name", f"set_{index:03d}"))
            extra_args = as_string_tuple(
                item.get("extra_args", []), f"param_grid.{method}[{index}].extra_args"
            )
            ignored = {"name", "params", "extra_args"}
            if "params" in item:
                unknown_top_level = sorted(set(item) - ignored)
                if unknown_top_level:
                    raise RuntimeError(
                        f"param_grid.{method}[{index}] has unknown keys: "
                        f"{', '.join(unknown_top_level)}"
                    )
            param_sets.append(ParamSet(name=slug(name), params=params, extra_args=extra_args))
        if not param_sets:
            raise RuntimeError(f"param_grid.{method} must not be empty")
        return param_sets
    if isinstance(raw_grid, dict):
        if "sets" in raw_grid:
            return parse_param_sets(raw_grid["sets"], method)
        if "grid" in raw_grid:
            grid_sets = expand_grid_object(as_param_dict(raw_grid["grid"], f"param_grid.{method}.grid"))
            prefix = str(raw_grid.get("name", method))
            return [
                ParamSet(name=slug(f"{prefix}_{item.name}"), params=item.params)
                for item in grid_sets
            ]
        return expand_grid_object(raw_grid)
    raise RuntimeError(f"param_grid.{method} must be a list or object")


def selected_keys(value: str, available: Sequence[str], label: str) -> list[str]:
    if value in {"default", "all"}:
        return list(available)
    requested = parse_csv_tokens(value)
    if not requested:
        raise RuntimeError(f"At least one {label} is required")
    unknown = [item for item in requested if item not in available]
    if unknown:
        raise RuntimeError(
            f"Unknown {label}(s): {', '.join(unknown)}. Available: {', '.join(available)}"
        )
    return requested


def build_specs(args: argparse.Namespace, config: dict[str, Any]) -> list[TrialSpec]:
    app_specs = parse_app_specs(config)
    app_keys = selected_keys(args.apps, tuple(app_specs), "app")
    specs: list[TrialSpec] = []
    for app_key in app_keys:
        app = app_specs[app_key]
        methods = selected_keys(args.methods, app.methods, "method")
        trials = args.trials_per_param_set or app.trials_per_param_set
        if trials < 1:
            raise RuntimeError("--trials-per-param-set must be at least 1")
        seeds = tuple(range(trials)) if args.trials_per_param_set else app.seeds
        assert seeds is not None
        for method in methods:
            raw_param_sets = app.param_grid.get(method)
            param_sets = parse_param_sets(raw_param_sets, method)
            for param_set in param_sets:
                params = merge_dicts(
                    app.base_params,
                    app.method_defaults.get(method, {}),
                    param_set.params,
                )
                if method == "parallel_arc":
                    params.setdefault(
                        "parallel_arc_conflict_find_assignment",
                        DEFAULT_PARALLEL_ARC_CONFLICT_FIND_ASSIGNMENT,
                    )
                merged_param_set = ParamSet(
                    name=param_set.name,
                    params=params,
                    extra_args=param_set.extra_args,
                )
                for seed in seeds:
                    specs.append(
                        TrialSpec(
                            app=app,
                            method=method,
                            param_set=merged_param_set,
                            seed=seed,
                            build_dir=args.build_dir,
                            output_root=args.output_root,
                            timeout_grace=args.timeout_grace,
                        )
                    )
    if not specs:
        raise RuntimeError("Sweep produced no trials")
    return specs


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


def valid_trial_record(record: Any) -> bool:
    return (
        isinstance(record, dict)
        and record.get("schema") == "comotion.param_sweep_trial.v1"
        and record.get("status") == "complete"
        and isinstance(record.get("result_row"), dict)
    )


def effective_time(metrics: dict[str, Any], time_limit: float, timed_out: bool) -> float:
    planning_time = finite_number(metrics.get("planning_time_seconds"))
    if planning_time is not None:
        return planning_time
    return time_limit if timed_out or not metrics.get("success") else 0.0


def result_row(
    spec: TrialSpec,
    metrics: dict[str, Any],
    returncode: int | None,
    timed_out: bool,
) -> dict[str, Any]:
    planning_time = finite_number(metrics.get("planning_time_seconds"))
    return {
        "app": spec.app.key,
        "app_title": spec.app.title,
        "method": spec.method,
        "method_label": spec.method_label,
        "param_set": spec.param_set.name,
        "seed": spec.seed,
        "success": bool(metrics.get("success")) and returncode == 0 and not timed_out,
        "planning_time_seconds": "" if planning_time is None else planning_time,
        "effective_planning_time_seconds": effective_time(
            metrics, spec.app.time_limit, timed_out
        ),
        "returncode": "" if returncode is None else returncode,
        "timed_out": timed_out,
        "trial_json": str(spec.record_path),
        "metrics_json": str(spec.metrics_path),
    }


def run_trial(
    spec: TrialSpec,
    *,
    existing: str,
    index: int,
    total: int,
) -> dict[str, Any]:
    if existing == "skip":
        existing_record = safe_load_json(spec.record_path)
        if valid_trial_record(existing_record):
            print(f"[{index}/{total}] skip {spec.trial_id}", flush=True)
            return existing_record

    spec.trial_dir.mkdir(parents=True, exist_ok=True)
    if existing == "overwrite" and spec.metrics_path.exists():
        spec.metrics_path.unlink()

    command = spec.command(spec.metrics_path)
    timeout_seconds = spec.app.time_limit + spec.timeout_grace
    print(f"[{index}/{total}] run {spec.trial_id}", flush=True)
    timed_out = False
    stdout = ""
    stderr = ""
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
        stdout = completed.stdout
        stderr = completed.stderr
    except subprocess.TimeoutExpired as exc:
        timed_out = True
        returncode = None
        stdout = text_tail(exc.stdout)
        stderr = text_tail(exc.stderr)

    metrics = safe_load_json(spec.metrics_path)
    row = result_row(spec, metrics, returncode, timed_out)
    record = {
        "schema": "comotion.param_sweep_trial.v1",
        "status": "complete",
        "completed_utc": utc_now(),
        "trial_id": spec.trial_id,
        "config_signature": spec.config_signature(),
        "command": command,
        "cwd": str(RUNTIME_CWD),
        "params": spec.param_set.params,
        "extra_args": list(spec.param_set.extra_args),
        "metrics": metrics,
        "returncode": returncode,
        "timed_out": timed_out,
        "timeout_seconds": timeout_seconds,
        "stdout_tail": text_tail(stdout),
        "stderr_tail": text_tail(stderr),
        "result_row": row,
    }
    atomic_write_json(spec.record_path, record)
    return record


def row_sort_key(row: dict[str, Any]) -> tuple[Any, ...]:
    return (
        row.get("app", ""),
        row.get("method", ""),
        row.get("param_set", ""),
        int(row.get("seed", 0)),
    )


def mean(values: Sequence[float]) -> float | None:
    if not values:
        return None
    return sum(values) / float(len(values))


def summarize(records: Sequence[dict[str, Any]]) -> tuple[list[dict[str, Any]], dict[tuple[str, str], dict[str, Any]]]:
    grouped: dict[tuple[str, str, str], list[dict[str, Any]]] = {}
    params_by_key: dict[tuple[str, str, str], dict[str, Any]] = {}
    extra_args_by_key: dict[tuple[str, str, str], list[str]] = {}
    for record in records:
        row = record["result_row"]
        key = (str(row["app"]), str(row["method"]), str(row["param_set"]))
        grouped.setdefault(key, []).append(row)
        params_by_key[key] = dict(record.get("params", {}))
        extra_args_by_key[key] = list(record.get("extra_args", []))

    summaries: list[dict[str, Any]] = []
    for key, rows in grouped.items():
        successful_times = [
            float(row["planning_time_seconds"])
            for row in rows
            if row.get("success") is True and row.get("planning_time_seconds") != ""
        ]
        effective_times = [
            float(row["effective_planning_time_seconds"])
            for row in rows
            if row.get("effective_planning_time_seconds") != ""
        ]
        app, method, param_set = key
        summaries.append(
            {
                "app": app,
                "method": method,
                "param_set": param_set,
                "trial_count": len(rows),
                "success_count": sum(1 for row in rows if row.get("success") is True),
                "mean_successful_planning_time_seconds": mean(successful_times),
                "mean_effective_planning_time_seconds": mean(effective_times),
                "params": params_by_key[key],
                "extra_args": extra_args_by_key[key],
            }
        )

    def rank_key(item: dict[str, Any]) -> tuple[Any, ...]:
        success_count = int(item["success_count"])
        successful_mean = item["mean_successful_planning_time_seconds"]
        effective_mean = item["mean_effective_planning_time_seconds"]
        return (
            -success_count,
            effective_mean if effective_mean is not None else float("inf"),
            successful_mean if successful_mean is not None else float("inf"),
            item["param_set"],
        )

    best_by_method: dict[tuple[str, str], dict[str, Any]] = {}
    for item in sorted(summaries, key=rank_key):
        method_key = (str(item["app"]), str(item["method"]))
        best_by_method.setdefault(method_key, item)
    for item in summaries:
        item["best"] = best_by_method[(str(item["app"]), str(item["method"]))] is item

    summaries.sort(
        key=lambda item: (
            item["app"],
            item["method"],
            not bool(item["best"]),
            rank_key(item),
        )
    )
    return summaries, best_by_method


def compact_summary_row(item: dict[str, Any]) -> dict[str, Any]:
    return {
        "app": item["app"],
        "method": item["method"],
        "param_set": item["param_set"],
        "trial_count": item["trial_count"],
        "success_count": item["success_count"],
        "mean_successful_planning_time_seconds": (
            "" if item["mean_successful_planning_time_seconds"] is None
            else item["mean_successful_planning_time_seconds"]
        ),
        "mean_effective_planning_time_seconds": (
            "" if item["mean_effective_planning_time_seconds"] is None
            else item["mean_effective_planning_time_seconds"]
        ),
        "best": item["best"],
    }


def write_outputs(
    args: argparse.Namespace,
    config: dict[str, Any],
    records: Sequence[dict[str, Any]],
) -> None:
    rows = sorted((record["result_row"] for record in records), key=row_sort_key)
    summaries, best_by_method = summarize(records)
    write_csv(args.output_root / "results.csv", RESULT_COLUMNS, rows)
    write_csv(
        args.output_root / "summary.csv",
        SUMMARY_COLUMNS,
        [compact_summary_row(item) for item in summaries],
    )

    summary_doc = {
        "schema": "comotion.param_sweep_summary.v1",
        "created_utc": utc_now(),
        "selection_rule": (
            "Select the highest success_count, then lowest mean effective "
            "planning_time_seconds, then lowest mean successful planning_time_seconds."
        ),
        "config": str(args.config),
        "output_root": str(args.output_root),
        "summaries": summaries,
    }
    atomic_write_json(args.output_root / "summary.json", summary_doc)

    best_doc: dict[str, Any] = {
        "schema": "comotion.best_planner_parameters.v1",
        "created_utc": utc_now(),
        "selection_rule": summary_doc["selection_rule"],
        "source_config": str(args.config),
        "source_output_root": str(args.output_root),
        "apps": {},
    }
    for (app, method), item in sorted(best_by_method.items()):
        app_doc = best_doc["apps"].setdefault(app, {"methods": {}})
        app_doc["methods"][method] = {
            "param_set": item["param_set"],
            "trial_count": item["trial_count"],
            "success_count": item["success_count"],
            "mean_successful_planning_time_seconds": item[
                "mean_successful_planning_time_seconds"
            ],
            "mean_effective_planning_time_seconds": item[
                "mean_effective_planning_time_seconds"
            ],
            "params": item["params"],
            "extra_args": item["extra_args"],
        }
    if args.best_params_output:
        atomic_write_json(args.best_params_output, best_doc)
    atomic_write_json(args.output_root / "best_params.json", best_doc)

    manifest = {
        "schema": "comotion.param_sweep_manifest.v1",
        "created_utc": utc_now(),
        "command_line": sys.argv,
        "config": str(args.config),
        "output_root": str(args.output_root),
        "build_dir": str(args.build_dir),
        "runtime_cwd": str(RUNTIME_CWD),
        "cores": args.cores,
        "existing": args.existing,
        "trial_count": len(records),
        "config_schema": config.get("schema"),
    }
    atomic_write_json(args.output_root / "manifest.json", manifest)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run a JSON-defined planner parameter sweep and choose parameters "
            "by successful planning_time_seconds."
        )
    )
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--apps", default="default")
    parser.add_argument("--methods", default="default")
    parser.add_argument("--trials-per-param-set", type=int)
    parser.add_argument("--cores", type=int, default=8)
    parser.add_argument("--build-dir", type=Path, default=DEFAULT_BUILD_DIR)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=None,
        help="Default: benchmarks/results/param_sweep_<UTC timestamp>.",
    )
    parser.add_argument("--best-params-output", type=Path)
    parser.add_argument("--existing", choices=("skip", "overwrite"), default="skip")
    parser.add_argument("--timeout-grace", type=float, default=30.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--dry-run-limit", type=int, default=50)
    args = parser.parse_args()
    if args.cores < 1:
        raise RuntimeError("--cores must be at least 1")
    if args.trials_per_param_set is not None and args.trials_per_param_set < 1:
        raise RuntimeError("--trials-per-param-set must be at least 1")
    if args.timeout_grace < 0.0:
        raise RuntimeError("--timeout-grace must be non-negative")
    if args.dry_run_limit < 0:
        raise RuntimeError("--dry-run-limit must be non-negative")
    args.output_root = args.output_root or DEFAULT_RESULTS_DIR / f"param_sweep_{timestamp()}"
    return args


def main() -> int:
    try:
        args = parse_args()
        config = read_json_object(args.config)
        specs = build_specs(args, config)

        if args.dry_run:
            limit = len(specs) if args.dry_run_limit == 0 else args.dry_run_limit
            for spec in specs[:limit]:
                print(" ".join(spec.command(spec.metrics_path)))
            if limit < len(specs):
                print(f"omitted_commands: {len(specs) - limit}")
            print(f"planned_trials: {len(specs)}")
            print(f"output_root: {args.output_root}")
            return 0

        records: list[dict[str, Any]] = []
        if args.cores == 1:
            for index, spec in enumerate(specs, start=1):
                records.append(
                    run_trial(spec, existing=args.existing, index=index, total=len(specs))
                )
        else:
            with concurrent.futures.ThreadPoolExecutor(max_workers=args.cores) as pool:
                futures = {
                    pool.submit(
                        run_trial,
                        spec,
                        existing=args.existing,
                        index=index,
                        total=len(specs),
                    ): spec
                    for index, spec in enumerate(specs, start=1)
                }
                for future in concurrent.futures.as_completed(futures):
                    records.append(future.result())

        write_outputs(args, config, records)
        print(f"manifest: {args.output_root / 'manifest.json'}")
        print(f"results_csv: {args.output_root / 'results.csv'}")
        print(f"summary_csv: {args.output_root / 'summary.csv'}")
        print(f"summary_json: {args.output_root / 'summary.json'}")
        print(f"best_params_json: {args.output_root / 'best_params.json'}")
        if args.best_params_output:
            print(f"best_params_output: {args.best_params_output}")
        return 0
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
