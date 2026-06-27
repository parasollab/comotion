# Benchmarks

The public benchmark surface is intentionally small:

- feasibility: cumulative exact-solution success over runtime
- anytime: cumulative success plus median current-best makespan over runtime
- multi-core: feasibility runs comparing ARC and ParallelARC worker counts

All workload executables are built from `apps/` and placed in `build/apps/`.
`benchmarks/` contains the runners, tests, and output documentation.

## Runner Requirements

The benchmark runners require Python 3.10 or newer. Trial execution, manifests,
and CSV outputs use only the Python standard library. Install matplotlib if you
want the runners to also produce PNG/SVG plots; when matplotlib is missing, the
runners skip plots and still write the machine-readable outputs.

Multi-core runs exercise ParallelARC's POSIX process-based worker path, so they
are intended for Linux-style environments with `fork`, pipes, and signals.

## Build

```bash
cmake -S . -B build -DCOMOTION_BUILD_APPS=ON
cmake --build build --target mobile_robot_2d_crossing
cmake --build build --target planar_manipulator_cross
cmake --build build --target panda_cage
cmake --build build --target panda_flat
```

## Public Runners

When run from the source tree, the runners default to `build/apps` for
executables and `benchmarks/results/<experiment>_<timestamp>` for outputs. When
run from an installed package, they default to the install prefix's `bin/`
directory for executables and `./comotion-results/<experiment>_<timestamp>` for
outputs. Use `--build-dir` and `--output-root` to override either location.
Installed runners execute workload apps from the install prefix's
`share/comotion/` directory so installed Panda and Planar3 resource paths resolve.

Installed runner example:

```bash
python3 /path/to/comotion-install/share/comotion/benchmarks/scripts/run_feasibility.py \
  --cases default \
  --num-seeds 5 \
  --time-limit 60
```

### Feasibility

```bash
python3 benchmarks/scripts/run_feasibility.py \
  --cases default \
  --algorithms arc,prioritized,drrt,stcbs \
  --num-seeds 5 \
  --time-limit 60
```

This writes cumulative success plots under `<output-root>/plots/`.

### Anytime

```bash
python3 benchmarks/scripts/run_anytime.py \
  --cases default \
  --algorithms arc,ao_arc,composite_aorrtc \
  --num-seeds 5 \
  --time-limit 120
```

This writes one stacked panel per benchmark case. The top panel is cumulative
success over runtime; the bottom panel is median current-best makespan over
runtime.

### Multi-Core

```bash
python3 benchmarks/scripts/run_multicore.py \
  --cases mobile_parallel_n8,planar_cross_n8 \
  --worker-counts 2,4 \
  --num-seeds 5 \
  --time-limit 60
```

This compares ARC against ParallelARC with the requested worker-process counts
and writes cumulative success plots.

By default, ParallelARC plans different robots in parallel during initial
solution construction but does not duplicate a robot's initial query. Add
`--parallel-arc-initial-solution-or` to any runner, or to a direct
`--algorithm parallel_arc` executable run, to let spare initial-planning workers
race duplicate attempts and keep the first successful path.

## Case Catalog

The runners expose these benchmark cases:

```text
mobile_parallel_n4
mobile_parallel_n8
mobile_parallel_n16
mobile_circle_n4
mobile_circle_n8
planar_cross_n4
planar_cross_n8
planar_adaptive_n8
panda_cage_n2
panda_cage_n4
panda_flat_n4
```

`--cases default` expands to a small smoke/evaluation set. Panda cases are
task-based and use `--task-indices` to select built-in tasks.

## Output Contract

Each runner writes:

```text
manifest.json
results.csv
solution_events.csv
plots/*.png
plots/*.svg
```

`results.csv` contains one row per trial. `solution_events.csv` contains the
solution-improvement events needed to regenerate anytime plots. The runners use
temporary per-trial metrics files internally, then discard them after extracting
the CSV rows needed by the public plots.

`results.csv` columns:

```text
case,case_title,task_index,seed,method,time_limit_seconds,success,first_solution_time_seconds
```

`solution_events.csv` columns:

```text
case,task_index,seed,method,elapsed_seconds,makespan_timesteps
```

Direct `--metrics-json` output from each workload executable uses the compact
schema consumed by the runners:

```json
{
  "success": true,
  "planning_time_seconds": 1.23,
  "makespan_timesteps": 42,
  "planner_stats": {
    "solution_events": [
      {
        "elapsed_seconds": 1.23,
        "makespan_timesteps": 42
      }
    ]
  }
}
```

## Direct Executable Use

The workload executables can also be called directly. For example:

```bash
./build/apps/mobile_robot_2d_crossing \
  --scenario parallel \
  --num-robots 4 \
  --algorithm arc \
  --collision-backend vamp \
  --time-limit 60 \
  --seed 0 \
  --metrics-json /tmp/mobile_parallel_n4_arc_seed0.json

./build/apps/planar_manipulator_cross \
  --scenario cross \
  --num-robots 4 \
  --algorithm arc \
  --collision-backend vamp \
  --time-limit 60 \
  --seed 0 \
  --metrics-json /tmp/planar_cross_n4_arc_seed0.json

./build/apps/panda_cage \
  --num-robots 2 \
  --task-index 0 \
  --algorithm arc \
  --collision-backend vamp \
  --time-limit 60 \
  --seed 0 \
  --metrics-json /tmp/panda_cage_n2_arc_seed0.json
```

Installed direct Panda or Planar3 executable runs resolve the packaged
resources relative to the installed `bin/` directory. Explicit resource paths
under `/path/to/comotion-install/share/comotion/resources/` can still be passed when
overriding the packaged models.
