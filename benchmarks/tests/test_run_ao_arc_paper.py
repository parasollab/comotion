#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
import unittest
from collections import Counter
from pathlib import Path
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = REPO_ROOT / "benchmarks" / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

import benchmark_runner_common as common  # noqa: E402
import run_ao_arc_paper as runner  # noqa: E402


class AoArcPaperRunnerTests(unittest.TestCase):
    def make_args(self, **overrides: object) -> argparse.Namespace:
        args = runner.parse_args([])
        for name, value in overrides.items():
            setattr(args, name, value)
        return args

    def build_default_specs(self):
        return runner.build_paper_specs(
            self.make_args(), output_root=Path("/tmp/ao-arc-paper-test")
        )

    def test_default_matrix_matches_the_paper(self) -> None:
        cases, variants, specs = self.build_default_specs()

        self.assertEqual([case.key for case in cases], list(runner.PAPER_CASE_KEYS))
        self.assertEqual(
            [variant.label for variant in variants],
            [
                "ARC",
                "AO-ARC",
                "CompRRTC",
                "CompAORRTC",
                "dRRT*",
                "PP-ST-RRT*",
            ],
        )
        self.assertEqual(len(specs), 1140)
        self.assertEqual(
            Counter((spec.case.key, spec.time_limit) for spec in specs),
            Counter(
                {
                    **{
                        (key, 300.0): 10 * 6
                        for key in runner.PAPER_2D_CASE_KEYS
                    },
                    **{
                        (key, 600.0): 5 * 10 * 6
                        for key in runner.PAPER_PANDA_CASE_KEYS
                    },
                }
            ),
        )
        self.assertEqual(
            len({(spec.case.key, spec.task_index) for spec in specs}),
            19,
        )

    def test_every_configuration_has_ten_trials_for_all_six_methods(self) -> None:
        _, variants, specs = self.build_default_specs()
        method_labels = {variant.label for variant in variants}
        grouped: dict[tuple[str, int | None], list] = {}
        for spec in specs:
            grouped.setdefault((spec.case.key, spec.task_index), []).append(spec)

        for config, config_specs in grouped.items():
            with self.subTest(config=config):
                self.assertEqual({spec.seed for spec in config_specs}, set(range(10)))
                self.assertEqual(
                    {spec.variant.label for spec in config_specs}, method_labels
                )
                self.assertEqual(len(config_specs), 10 * 6)

    def test_panda_task_generation_seed_is_fixed_across_trial_seeds(self) -> None:
        _, _, specs = runner.build_paper_specs(
            self.make_args(
                cases="panda_cage_n4",
                methods="arc",
                seeds="3,9",
                task_indices="2",
                task_generation_seed=41,
            ),
            output_root=Path("/tmp/ao-arc-paper-test"),
        )
        self.assertEqual(len(specs), 2)
        for spec in specs:
            command = spec.command()
            self.assertEqual(
                command[command.index("--task-generation-seed") + 1], "41"
            )
            self.assertEqual(command[command.index("--seed") + 1], str(spec.seed))

    def test_paper_method_flags_select_the_intended_algorithms(self) -> None:
        args = self.make_args(
            cases="mobile_parallel_n4", seeds="0", task_indices="0"
        )
        _, _, specs = runner.build_paper_specs(
            args, output_root=Path("/tmp/ao-arc-paper-test")
        )
        commands = {spec.variant.label: spec.command() for spec in specs}

        self.assertIn("--arc-local-composite-use-makespan-metric", commands["ARC"])
        self.assertIn(
            "--arc-local-composite-use-makespan-metric", commands["AO-ARC"]
        )
        ao_arc_command = commands["AO-ARC"]
        self.assertEqual(
            ao_arc_command[
                ao_arc_command.index(
                    "--ao-arc-repair-history-replanning-depth"
                )
                + 1
            ],
            "0",
        )
        self.assertEqual(
            ao_arc_command[
                ao_arc_command.index(
                    "--ao-arc-random-full-restart-probability"
                )
                + 1
            ],
            "0",
        )
        self.assertIn("--composite-rrt-use-makespan-metric", commands["CompRRTC"])
        self.assertEqual(
            commands["dRRT*"][commands["dRRT*"].index("--drrt-cost-metric") + 1],
            "makespan",
        )
        pp_command = commands["PP-ST-RRT*"]
        self.assertIn("--strrt-shuffle-priority-order", pp_command)
        self.assertEqual(
            pp_command[pp_command.index("--strrt-return-first-solution") + 1],
            "0",
        )
        self.assertEqual(
            pp_command[pp_command.index("--strrt-rewiring") + 1], "knearest"
        )

    def test_per_trial_timeout_uses_each_specs_time_limit(self) -> None:
        _, _, specs = self.build_default_specs()
        selected = [
            next(spec for spec in specs if not spec.case.task_based),
            next(spec for spec in specs if spec.case.task_based),
        ]
        observed: list[float | None] = []

        def fake_run_trial(spec, timeout_seconds, **_kwargs):
            observed.append(timeout_seconds)
            return ({"case": spec.case.key}, [])

        with mock.patch.object(common, "run_trial", side_effect=fake_run_trial):
            common.run_trials(
                selected,
                jobs=1,
                timeout_seconds=None,
                timeout_grace_seconds=30.0,
                skip_existing=False,
            )

        self.assertEqual(observed, [330.0, 630.0])


if __name__ == "__main__":
    unittest.main()
