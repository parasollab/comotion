# Benchmarks

CoMotion provides:

- method-based VA-MRMP comparisons using VAMP, sphere, and FCL collision
  backends;
- final P-ARC paper reproduction runners for 2D and Panda Cage scenarios;
- P-ARC conflict-detection and optimistic-batching ablations;
- smaller feasibility, anytime, and multi-core experiments


## Requirements

Follow the main [installation instructions](README.md#installation). Benchmark
runners require Python 3.10 or newer. Install matplotlib to generate PNG and SVG
plots; CSV and JSON output does not require it.

Build the applications before running experiments:

```bash
cmake -S . -B build
cmake --build build
```

Run commands from the repository root. Use `--help` on any runner for its full
set of cases and options.

## VA-MRMP Method Comparisons

The planner-trial runner supports `panda_cage`, `flying_spheres`, and
`heterogeneous_corridor` with VAMP, sphere, and FCL collision backends.

Preview the complete default matrix:

```bash
python3 benchmarks/scripts/run_planner_trials.py \
  --backends vamp,sphere,fcl \
  --dry-run
```

Run it with eight concurrent top-level trials:

```bash
python3 benchmarks/scripts/run_planner_trials.py \
  --scenarios default \
  --methods default \
  --backends vamp,sphere,fcl \
  --cores 8 \
  --output-root benchmarks/results/va_mrmp
```

The runner uses `benchmarks/configs/planner_trial_params.json` and prunes a
method/backend/task combination at larger team sizes after it fails every trial
at a smaller size. Reuse the same `--output-root` to resume compatible trials.

## P-ARC Paper Reproduction

Inspect the final command matrices without running trials:

```bash
python3 benchmarks/scripts/run_parallel_arc_2d.py --dry-run
python3 benchmarks/scripts/run_parallel_arc_panda.py --dry-run
```

Run the final 2D and Panda Cage experiments:

```bash
python3 benchmarks/scripts/run_parallel_arc_2d.py \
  --output-root benchmarks/results/parallel_arc_2d

python3 benchmarks/scripts/run_parallel_arc_panda.py \
  --output-root benchmarks/results/parallel_arc_panda
```

These runners encode the final paper parameters. The same profiles are the
application defaults for the corresponding Mobile, Planar Cross, and Panda
Cage scenarios.

### P-ARC Ablations

Preview the conflict-detection horizon ablation:

```bash
python3 benchmarks/scripts/run_parallel_arc_conflict_ablation.py --dry-run
```

Run the final eight-robot Panda Cage optimistic-batching matrix:

```bash
python3 benchmarks/scripts/run_parallel_arc_optimistic_ablation.py \
  --num-seeds 10 \
  --task-indices 0,1,2,3,4 \
  --time-limit 100 \
  --jobs 1
```

Use `--allow-nonpaper-matrix` only for smaller diagnostic runs.

Use the P-ARC runners above, not `run_planner_trials.py`, for final P-ARC paper
reproduction.

## Smaller Experiments

Run feasibility comparisons:

```bash
python3 benchmarks/scripts/run_feasibility.py \
  --cases default \
  --algorithms arc,prioritized,drrt,stcbs \
  --num-seeds 5 \
  --time-limit 60
```

Run anytime comparisons:

```bash
python3 benchmarks/scripts/run_anytime.py \
  --cases default \
  --algorithms arc,ao_arc,composite_aorrtc \
  --num-seeds 5 \
  --time-limit 120
```

Compare ARC and ParallelARC worker counts:

```bash
python3 benchmarks/scripts/run_multicore.py \
  --cases mobile_parallel_n8,planar_cross_n8 \
  --worker-counts 2,4 \
  --num-seeds 5 \
  --time-limit 60
```

## Results

Runners write:

```text
manifest.json
results.csv
solution_events.csv
plots/
```

`results.csv` contains one row per trial. `solution_events.csv` records
solution improvements for anytime plots.

Regenerate plots from an existing result:

```bash
python3 benchmarks/scripts/plot_results.py \
  path/to/results.csv \
  --plot-backends
```

Source-tree runs use `build/apps` and create timestamped directories beneath
`benchmarks/results/` by default. Use `--build-dir` and `--output-root` to
override those locations.

Run an installed benchmark runner with:

```bash
python3 /path/to/comotion-install/share/comotion/benchmarks/scripts/run_feasibility.py \
  --cases default \
  --num-seeds 5 \
  --time-limit 60
```
