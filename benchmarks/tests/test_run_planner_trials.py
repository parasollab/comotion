#!/usr/bin/env python3
"""Regression coverage for the paper-scale planner-trial runner."""

from __future__ import annotations

import argparse
import contextlib
import csv
import hashlib
import io
import json
import sys
import tempfile
import unittest
from collections import Counter
from pathlib import Path
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / "benchmarks" / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

import run_planner_trials as runner  # noqa: E402
import benchmark_runner_common as common  # noqa: E402


class PlannerTrialRunnerTest(unittest.TestCase):
    def make_args(self, **overrides: object) -> argparse.Namespace:
        values: dict[str, object] = {
            "scenarios": "default",
            "methods": "default",
            "backends": "default",
            "robot_counts": "default",
            "num_trials": None,
            "task_indices": "default",
            "time_limit": None,
            "time_limits": "",
            "resolution": 128,
            "build_dir": REPO_ROOT / "build" / "apps",
            "output_root": Path("/tmp/comotion-planner-trial-test-output"),
            "planner_params": (
                REPO_ROOT / "benchmarks" / "configs" / "planner_trial_params.json"
            ),
            "cores": 1,
            "existing": "skip",
            "timeout_grace": 0.0,
        }
        values.update(overrides)
        return argparse.Namespace(**values)

    def test_default_matrix_covers_every_requested_team_size(self) -> None:
        specs = runner.build_trial_specs(self.make_args())
        counts = Counter((spec.scenario.key, spec.num_robots) for spec in specs)

        expected: dict[tuple[str, int], int] = {}
        for count in (4, 8, 16, 32, 64, 128):
            expected[("flying_spheres", count)] = 30 * 5 * 3
        for count in (4, 8, 16):
            expected[("panda_cage", count)] = 50 * 5 * 3
            expected[("heterogeneous_corridor", count)] = 10 * 5 * 3

        self.assertEqual(counts, Counter(expected))
        self.assertEqual(len(specs), 5400)

    def test_default_time_limits_match_paper(self) -> None:
        self.assertEqual(
            runner.SCENARIOS["flying_spheres"].default_time_limit,
            10.0,
        )
        self.assertEqual(runner.SCENARIOS["panda_cage"].default_time_limit, 150.0)
        self.assertEqual(
            runner.SCENARIOS["heterogeneous_corridor"].default_time_limit,
            150.0,
        )

    def test_panda_sizes_use_five_tasks_and_ten_seeds_each(self) -> None:
        specs = runner.build_trial_specs(
            self.make_args(
                scenarios="panda_cage",
                methods="composite",
                backends="vamp",
            )
        )
        observed = Counter(
            (spec.num_robots, spec.task_index, spec.seed) for spec in specs
        )
        expected = Counter(
            (count, task_index, seed)
            for count in (4, 8, 16)
            for task_index in range(5)
            for seed in range(10)
        )
        self.assertEqual(observed, expected)

    def test_n8_parameter_profiles_are_extrapolated_across_sizes(self) -> None:
        param_doc = runner.read_json_file(
            REPO_ROOT / "benchmarks" / "configs" / "planner_trial_params.json"
        )
        for scenario_key in ("flying_spheres", "panda_cage"):
            scenario = runner.SCENARIOS[scenario_key]
            for method in runner.PAPER_METHODS:
                expected = runner.resolve_planner_params(
                    param_doc,
                    scenario=scenario,
                    num_robots=8,
                    method=method,
                )
                for count in scenario.robot_counts:
                    with self.subTest(
                        scenario=scenario_key, method=method, count=count
                    ):
                        actual = runner.resolve_planner_params(
                            param_doc,
                            scenario=scenario,
                            num_robots=count,
                            method=method,
                        )
                        self.assertEqual(actual, expected)

    def test_arc_paper_profiles_use_two_phase_exponential_expansion(self) -> None:
        param_doc = runner.read_json_file(
            REPO_ROOT / "benchmarks" / "configs" / "planner_trial_params.json"
        )
        expected_profiles = {
            "flying_spheres": {
                "arc_initial_window": 50,
                "arc_expansion_step": 2.0,
                "arc_initial_valid_expansion_step": 50,
            },
            "panda_cage": {
                "arc_initial_window": 20,
                "arc_expansion_step": 1.05,
                "arc_initial_valid_expansion_step": 20,
            },
            "heterogeneous_corridor": {
                "arc_initial_window": 20,
                "arc_expansion_step": 1.05,
                "arc_initial_valid_expansion_step": 20,
            },
        }
        for scenario_key, expected in expected_profiles.items():
            params = runner.resolve_planner_params(
                param_doc,
                scenario=runner.SCENARIOS[scenario_key],
                num_robots=runner.SCENARIOS[scenario_key].robot_counts[0],
                method="arc",
            )
            with self.subTest(scenario=scenario_key):
                self.assertEqual(params["arc_expansion_policy"], "exponential")
                self.assertEqual(
                    params["arc_initial_valid_expansion_policy"], "linear"
                )
                for key, value in expected.items():
                    self.assertEqual(params[key], value)

    def test_arc_expansion_policy_parameters_map_to_cli(self) -> None:
        args = runner.planner_params_to_args(
            {
                "arc_expansion_step": 1.5,
                "arc_expansion_policy": "multiplied",
                "arc_expansion_multipliers": "1,1,1,2,2,2,4,8",
                "arc_initial_valid_expansion_policy": "logarithmic",
                "arc_initial_valid_expansion_step": 10,
                "arc_initial_valid_expansion_multipliers": "1,2,4",
                "arc_initial_valid_expansion_symmetric": False,
            }
        )
        self.assertEqual(
            args,
            (
                "--arc-expansion-step",
                "1.5",
                "--arc-expansion-policy",
                "multiplied",
                "--arc-expansion-multipliers",
                "1,1,1,2,2,2,4,8",
                "--arc-initial-valid-expansion-policy",
                "logarithmic",
                "--arc-initial-valid-expansion-step",
                "10",
                "--arc-initial-valid-expansion-multipliers",
                "1,2,4",
                "--arc-initial-valid-asymmetric-expansion",
            ),
        )
        inherited_args = runner.planner_params_to_args(
            {
                "arc_expansion_policy": "logarithmic",
                "arc_expansion_step": 20,
            }
        )
        self.assertNotIn(
            "--arc-initial-valid-expansion-policy", inherited_args
        )
        self.assertNotIn(
            "--arc-initial-valid-expansion-step", inherited_args
        )
        self.assertNotIn(
            "--arc-initial-valid-expansion-multipliers", inherited_args
        )

    def test_vamp_validation_strategy_defaults_are_combined_rake(self) -> None:
        param_doc = runner.read_json_file(
            REPO_ROOT / "benchmarks" / "configs" / "planner_trial_params.json"
        )
        expected = {
            "arc": "combined_rake",
            "drrt": "combined_rake",
            "prioritized": "combined_rake",
            "stcbs": "combined_rake",
            "composite": "combined_rake",
        }
        for method, strategy in expected.items():
            with self.subTest(method=method):
                params = runner.resolve_planner_params(
                    param_doc,
                    scenario=runner.SCENARIOS["panda_cage"],
                    num_robots=8,
                    method=method,
                )
                self.assertEqual(params["vamp_validation_strategy"], strategy)
                self.assertIn(
                    "--vamp-validation-strategy",
                    runner.planner_params_to_args(params),
                )

    def test_parallel_arc_uses_cyclic_cover_greedy_by_default(self) -> None:
        param_doc = runner.read_json_file(
            REPO_ROOT / "benchmarks" / "configs" / "planner_trial_params.json"
        )
        params = runner.resolve_planner_params(
            param_doc,
            scenario=runner.SCENARIOS["panda_cage"],
            num_robots=8,
            method="parallel_arc",
        )
        self.assertEqual(
            params["parallel_arc_conflict_find_assignment"],
            "cyclic_cover_greedy",
        )
        args = runner.planner_params_to_args(params)
        assignment_index = args.index(
            "--parallel-arc-conflict-find-assignment"
        )
        self.assertEqual(args[assignment_index + 1], "cyclic_cover_greedy")

        common_args = common.effective_variant_extra_args(
            common.PlannerVariant(
                label="P-ARC-8",
                algorithm="parallel_arc",
                slug="p_arc_8",
            )
        )
        self.assertEqual(
            common_args,
            (
                "--parallel-arc-conflict-find-assignment",
                "cyclic_cover_greedy",
            ),
        )

    def test_parallel_arc_assignment_override_is_preserved(self) -> None:
        variant = common.PlannerVariant(
            label="P-ARC-8 round robin",
            algorithm="parallel_arc",
            slug="p_arc_8_round_robin",
            extra_args=(
                "--parallel-arc-conflict-find-assignment",
                "round_robin",
            ),
        )
        effective = common.effective_variant_extra_args(variant)
        self.assertEqual(
            effective.count("--parallel-arc-conflict-find-assignment"),
            1,
        )
        self.assertEqual(effective[-1], "round_robin")

    def test_parallel_trial_batch_assigns_cores_round_robin(self) -> None:
        specs = runner.build_trial_specs(
            self.make_args(
                scenarios="flying_spheres",
                methods="arc",
                backends="vamp",
                robot_counts="4",
                num_trials=5,
                cores=3,
            )
        )
        seen: list[int | None] = []

        def fake_run_one_trial(
            spec: runner.TrialSpec,
            **kwargs: object,
        ) -> dict[str, object]:
            del spec
            seen.append(kwargs.get("pinned_core"))  # type: ignore[arg-type]
            return {"result_row": {}, "event_rows": []}

        with mock.patch.object(runner, "available_worker_cores", return_value=[10, 11, 12]):
            with mock.patch.object(runner, "run_one_trial", side_effect=fake_run_one_trial):
                runner.run_trial_batch(
                    specs,
                    args=self.make_args(cores=3),
                    first_index=1,
                    total=len(specs),
                )

        self.assertEqual(Counter(seen), Counter({10: 2, 11: 2, 12: 1}))

    def test_default_backend_order_supports_validation_cascade(self) -> None:
        self.assertEqual(
            runner.resolve_backends("default"),
            ["vamp", "sphere", "fcl"],
        )

    def test_vamp_timeout_skips_only_later_backends_in_same_group(self) -> None:
        specs = runner.build_trial_specs(
            self.make_args(
                scenarios="panda_cage",
                methods="stcbs,drrt",
                backends="default",
                robot_counts="4",
                num_trials=2,
                task_indices="0,1",
            )
        )
        calls: list[tuple[int | None, int, str, str]] = []

        def fake_run_one_trial(
            spec: runner.TrialSpec,
            **kwargs: object,
        ) -> dict[str, object]:
            del kwargs
            calls.append(
                (spec.task_index, spec.seed, spec.method, spec.backend)
            )
            timed_out = (
                spec.task_index == 0
                and spec.seed == 0
                and spec.method == "stcbs"
                and spec.backend == "vamp"
            )
            return {
                "returncode": 0,
                "timed_out": timed_out,
                "result_row": {"success": not timed_out},
                "event_rows": [],
            }

        with mock.patch.object(runner, "available_worker_cores", return_value=[0]):
            with mock.patch.object(
                runner, "run_one_trial", side_effect=fake_run_one_trial
            ):
                with contextlib.redirect_stdout(io.StringIO()):
                    runner.run_trial_batch(
                        specs,
                        args=self.make_args(),
                        first_index=1,
                        total=len(specs),
                    )

        self.assertIn((0, 0, "stcbs", "vamp"), calls)
        self.assertNotIn((0, 0, "stcbs", "sphere"), calls)
        self.assertNotIn((0, 0, "stcbs", "fcl"), calls)
        for backend in ("vamp", "sphere", "fcl"):
            self.assertIn((0, 0, "drrt", backend), calls)
            self.assertIn((1, 0, "stcbs", backend), calls)

    def test_sphere_timeout_skips_fcl_but_not_other_groups(self) -> None:
        specs = runner.build_trial_specs(
            self.make_args(
                scenarios="panda_cage",
                methods="stcbs",
                backends="default",
                robot_counts="4",
                num_trials=1,
                task_indices="0",
            )
        )
        calls: list[str] = []

        def fake_run_one_trial(
            spec: runner.TrialSpec,
            **kwargs: object,
        ) -> dict[str, object]:
            del kwargs
            calls.append(spec.backend)
            timeout = spec.backend == "sphere"
            return {
                "returncode": 0,
                "timed_out": False,
                "result_row": {"success": not timeout},
                "event_rows": [],
            }

        with mock.patch.object(runner, "available_worker_cores", return_value=[0]):
            with mock.patch.object(
                runner, "run_one_trial", side_effect=fake_run_one_trial
            ):
                with contextlib.redirect_stdout(io.StringIO()):
                    runner.run_trial_batch(
                        specs,
                        args=self.make_args(),
                        first_index=1,
                        total=len(specs),
                    )

        self.assertEqual(calls, ["vamp", "sphere"])

    def test_heterogeneous_commands_and_labels_use_paired_counts(self) -> None:
        specs = runner.build_trial_specs(
            self.make_args(
                scenarios="heterogeneous_corridor",
                methods="composite",
                backends="vamp",
                num_trials=1,
            )
        )
        self.assertEqual(len(specs), 3)
        paired_counts = zip(specs, (4, 8, 16), (8, 16, 32))
        for spec, panda_count, sphere_count in paired_counts:
            command = spec.command()
            self.assertEqual(
                command[command.index("--num-pandas") + 1], str(panda_count)
            )
            self.assertEqual(
                command[command.index("--num-spheres") + 1], str(sphere_count)
            )
            task_file = command[command.index("--task-file") + 1]
            self.assertEqual(
                task_file,
                dict(spec.scenario.task_files)[panda_count],
            )
            self.assertTrue((REPO_ROOT / task_file).is_file())
            self.assertEqual(
                spec.case_key,
                f"heterogeneous_corridor_p{panda_count}_s{sphere_count}",
            )
            self.assertIn(
                f"{panda_count} Pandas + {sphere_count} spheres", spec.case_title
            )
            row = runner.build_result_row(
                spec=spec,
                metrics={},
                returncode=1,
                timed_out=False,
            )
            self.assertEqual(row["num_robots"], panda_count + sphere_count)
            self.assertEqual(row["primary_robot_count"], panda_count)
            self.assertEqual(row["secondary_robot_count"], sphere_count)

        timed_row = runner.build_result_row(
            spec=specs[0],
            metrics={"validation_time_seconds": 1.25},
            returncode=0,
            timed_out=False,
        )
        self.assertIn("validation_time_seconds", runner.RESULT_COLUMNS)
        self.assertEqual(timed_row["validation_time_seconds"], 1.25)

    def test_heterogeneous_tasks_match_canonical_mr_ompl_assets(self) -> None:
        expected = {
            4: (
                8,
                "9b146357c4766dca4d197137979229a6558e3b7a02dd5b6831b08de493db64a3",
                [-0.35, -0.35, 0.0],
            ),
            8: (
                16,
                "6d6d6cbe8cf87fa2872d4cae196bb9633bac4b256d0295670ff3de7e550f7f26",
                [-1.05, -0.35, 0.0],
            ),
            16: (
                32,
                "c371a5a984e7c2650e81553a0f9d4e7780b51ec333e39892f5bb8b4adf0b7729",
                [-2.45, -0.35, 0.0],
            ),
        }
        task_files = dict(runner.SCENARIOS["heterogeneous_corridor"].task_files)
        for panda_count, (sphere_count, digest, first_base) in expected.items():
            path = REPO_ROOT / task_files[panda_count]
            raw = path.read_bytes()
            doc = json.loads(raw)
            robots = doc["robots"]
            pandas = [robot for robot in robots if robot["type"] == "panda"]
            spheres = [robot for robot in robots if robot["type"] == "sphere"]

            self.assertEqual(hashlib.sha256(raw).hexdigest(), digest)
            self.assertEqual(len(pandas), panda_count)
            self.assertEqual(len(spheres), sphere_count)
            self.assertEqual(pandas[0]["base"]["position"], first_base)
            self.assertTrue(all(robot["radius"] == 0.1 for robot in spheres))
            self.assertEqual(doc["obstacles"], [])
            self.assertEqual(
                len(doc["tasks"][0]["starts"]), panda_count + sphere_count
            )
            self.assertEqual(
                [robot["type"] for robot in robots],
                ["panda"] * panda_count + ["sphere"] * sphere_count,
            )

    def test_heterogeneous_prioritized_profile_matches_mr_ompl(self) -> None:
        param_doc = runner.read_json_file(
            REPO_ROOT / "benchmarks" / "configs" / "planner_trial_params.json"
        )
        params = runner.resolve_planner_params(
            param_doc,
            scenario=runner.SCENARIOS["heterogeneous_corridor"],
            num_robots=8,
            method="prioritized",
        )
        self.assertEqual(params["strrt_initial_batch_size"], 4096)
        self.assertEqual(params["strrt_initial_time_factor"], 2.0)
        self.assertTrue(params["strrt_shuffle_priority_order"])
        self.assertIn(
            "--strrt-shuffle-priority-order",
            runner.planner_params_to_args(params),
        )

    def test_priority_shuffle_can_be_disabled_explicitly(self) -> None:
        args = runner.planner_params_to_args(
            {"strrt_shuffle_priority_order": False}
        )
        self.assertEqual(args, ("--no-strrt-shuffle-priority-order",))

    def test_resume_skip_requires_matching_config_signature(self) -> None:
        spec = runner.build_trial_specs(
            self.make_args(
                scenarios="flying_spheres",
                methods="composite",
                backends="vamp",
                robot_counts="4",
                num_trials=1,
            )
        )[0]
        record = {
            "schema": "comotion.planner_trial_record.v1",
            "status": "complete",
            "config_signature": spec.config_signature(),
            "result_row": runner.build_result_row(
                spec=spec,
                metrics={},
                returncode=1,
                timed_out=False,
            ),
            "event_rows": [],
        }

        self.assertTrue(runner.reusable_trial_record(record, spec))
        stale_record = {**record, "config_signature": "stale"}
        with self.assertRaisesRegex(RuntimeError, "--existing overwrite"):
            runner.reusable_trial_record(stale_record, spec)
        missing_signature = dict(record)
        missing_signature.pop("config_signature")
        with self.assertRaisesRegex(RuntimeError, "--existing overwrite"):
            runner.reusable_trial_record(missing_signature, spec)

    def test_incremental_outputs_merge_existing_trial_records(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output_root = Path(temp_dir)
            specs = runner.build_trial_specs(
                self.make_args(
                    scenarios="flying_spheres",
                    methods="arc",
                    backends="vamp",
                    robot_counts="4,8",
                    num_trials=1,
                    output_root=output_root,
                )
            )
            existing_spec = next(spec for spec in specs if spec.num_robots == 4)
            new_spec = next(spec for spec in specs if spec.num_robots == 8)
            existing_row = runner.build_result_row(
                spec=existing_spec,
                metrics={},
                returncode=1,
                timed_out=False,
            )
            existing_row.pop("validation_time_seconds")
            runner.atomic_write_json(
                existing_spec.record_path,
                {
                    "schema": "comotion.planner_trial_record.v1",
                    "status": "complete",
                    "trial_id": existing_spec.trial_id,
                    "result_row": existing_row,
                    "event_rows": [],
                },
            )
            new_record = {
                "schema": "comotion.planner_trial_record.v1",
                "status": "complete",
                "trial_id": new_spec.trial_id,
                "result_row": runner.build_result_row(
                    spec=new_spec,
                    metrics={"validation_time_seconds": 0.5},
                    returncode=0,
                    timed_out=False,
                ),
                "event_rows": [],
            }

            aggregate = runner.merge_trial_records(
                runner.load_aggregate_trial_records(output_root),
                [new_record],
            )
            runner.write_outputs(output_root, aggregate)

            with (output_root / "results.csv").open(newline="") as handle:
                rows = list(csv.DictReader(handle))

        self.assertEqual(len(rows), 2)
        by_count = {int(row["num_robots"]): row for row in rows}
        self.assertEqual(by_count[4]["validation_time_seconds"], "")
        self.assertEqual(by_count[8]["validation_time_seconds"], "0.5")

    def test_manifest_uses_suite_defaults_and_explicit_count_fields(self) -> None:
        args = self.make_args(
            scenarios="heterogeneous_corridor",
            methods="composite",
            backends="vamp",
            num_trials=1,
        )
        specs = runner.build_trial_specs(args)
        selected_scenarios = runner.resolve_scenarios(args.scenarios)
        methods = runner.resolve_methods(args.methods)
        backends = runner.resolve_backends(args.backends)

        with tempfile.TemporaryDirectory() as temp_dir:
            output_root = Path(temp_dir)
            runner.write_manifest(
                output_root,
                args=args,
                specs=specs,
                selected_scenarios=selected_scenarios,
                methods=methods,
                backends=backends,
            )
            with (output_root / "manifest.json").open() as handle:
                manifest = json.load(handle)

        self.assertIn("suite_defaults", manifest)
        self.assertNotIn("paper_defaults", manifest)
        scenario_doc = manifest["scenarios"][0]
        self.assertEqual(scenario_doc["primary_robot_counts"], [4, 8, 16])
        self.assertEqual(scenario_doc["total_robot_counts"], [12, 24, 48])
        suite_doc = manifest["suite_defaults"]["heterogeneous_corridor"]
        self.assertEqual(suite_doc["primary_robot_counts"], [4, 8, 16])
        self.assertEqual(suite_doc["total_robot_counts"], [12, 24, 48])

    def test_pruning_is_task_method_and_backend_scoped(self) -> None:
        specs = runner.build_trial_specs(
            self.make_args(
                scenarios="panda_cage",
                methods="composite",
                backends="vamp,fcl",
                robot_counts="4,8",
                num_trials=4,
                task_indices="0,1",
            )
        )
        calls: list[list[runner.TrialSpec]] = []

        def fake_batch(
            batch_specs: list[runner.TrialSpec],
            *,
            args: argparse.Namespace,
            first_index: int,
            total: int,
        ) -> list[dict[str, object]]:
            del args, first_index, total
            calls.append(list(batch_specs))
            records: list[dict[str, object]] = []
            for spec in batch_specs:
                success = not (spec.task_index == 0 and spec.backend == "vamp")
                records.append(
                    {
                        "result_row": {
                            "task_index": spec.task_index,
                            "algorithm": spec.method,
                            "collision_backend": spec.backend,
                            "success": success,
                        },
                        "event_rows": [],
                    }
                )
            return records

        with mock.patch.object(runner, "run_trial_batch", side_effect=fake_batch):
            with contextlib.redirect_stdout(io.StringIO()):
                records, events = runner.run_trials_with_pruning(
                    list(reversed(specs)), args=self.make_args()
                )

        self.assertEqual([batch[0].num_robots for batch in calls], [4, 8])
        active_at_eight = {
            (spec.task_index, spec.method, spec.backend) for spec in calls[1]
        }
        self.assertNotIn((0, "composite", "vamp"), active_at_eight)
        self.assertIn((0, "composite", "fcl"), active_at_eight)
        self.assertIn((1, "composite", "vamp"), active_at_eight)
        self.assertEqual(len(records), 14)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["task_index"], 0)
        self.assertEqual(events[0]["method"], "composite")
        self.assertEqual(events[0]["backend"], "vamp")
        self.assertEqual(events[0]["trial_count"], 2)
        self.assertEqual(events[0]["pruned_larger_robot_counts"], [8])


if __name__ == "__main__":
    unittest.main()
