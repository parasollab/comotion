# CoMotion

CoMotion, short for Coordinated Motion Planning for Multi-Robot Systems, is a
C++ multi-robot motion planning library and benchmark suite built on the
CoMotion forks of OMPL and VAMP. CoMotion extends the CPU SIMD-based
acceleration for single-robot motion planning introduced in VAMP to the
multi-robot domain, which we refer to as "Vector-Accelerated Multi-Robot Motion
Planning (VA-MRMP)."

## Citation

If you found this research useful for your own work, please use the following
citation:

```bibtex
@article{motes2026multi,
  title={Multi-Robot Motions in Milliseconds: Vector-Accelerated Primitives for Sampling-Based Planning},
  author={Motes, James D and Morales, Marco and Amato, Nancy M},
  journal={arXiv preprint arXiv:2604.23960},
  year={2026},
  url={https://doi.org/10.48550/arXiv.2604.23960}
}
```

For work using P-ARC, OR-ARC, or OR-P-ARC, also cite:

```bibtex
@article{motes2026parc,
  title={P-ARC: Exploiting Subproblem Independence for Parallel Multi-Robot Motion Planning},
  author={Motes, James D and Morales, Marco and Amato, Nancy M},
  journal={arXiv preprint arXiv:2606.27625},
  year={2026},
  url={https://doi.org/10.48550/arXiv.2606.27625}
}
```

For work using AO-ARC or Composite AORRTC, also cite:

```bibtex
@article{motes2026aoarc,
  title={AO-ARC: Almost-Surely Asymptotically Optimal Multi-Robot Motion Planning with ARC},
  author={Motes, James D and Morales, Marco and Amato, Nancy M},
  journal={arXiv preprint arXiv:2606.27495},
  year={2026},
  url={https://doi.org/10.48550/arXiv.2606.27495}
}
```

The same citation metadata is available in [CITATION.cff](CITATION.cff).

## Installation

CoMotion currently targets Linux C++ research environments. ParallelARC and
the multi-core benchmark runners use POSIX process primitives such as
`fork`, pipes, and signals.

### System Dependencies

Install the following dependencies before configuring CoMotion:

- A C++17 compiler and CMake 3.14 or newer.
- Git with submodule support.
- Boost.Serialization and Boost.Program_options.
- Eigen3.
- FCL and `pkg-config`. FCL is required to configure and build CoMotion.
- nlohmann/json 3.11.3 or newer. CMake fetches it automatically when a suitable
  system package is unavailable.
- Python 3.10 or newer for the benchmark runners.
- matplotlib, optional, for benchmark PNG and SVG plots. Benchmark CSV and JSON
  output does not require it.

VAMP selects architecture-specific SIMD flags by default. Set `VAMP_ARCH`
when portable binaries are needed for a different CPU target.

### Git Cloning

Clone CoMotion and all nested dependencies over HTTPS:

```bash
git clone --recurse-submodules https://github.com/parasollab/comotion.git
cd comotion
```

For an existing clone created without `--recurse-submodules`, initialize the
bundled dependencies before configuring:

```bash
git submodule update --init --recursive
```

CoMotion builds the bundled CoMotion OMPL fork from `external/como-ompl`.
That fork builds the bundled CoMotion VAMP fork from
`external/como-ompl/external/vamp`.

### Build Instructions

Configure and build the library, applications, examples, and tests:

```bash
cmake -S . -B build
cmake --build build
```

The standalone build enables applications, examples, and tests by default.
They can also be selected explicitly:

```bash
cmake -S . -B build \
  -DCOMOTION_BUILD_APPS=ON \
  -DCOMOTION_BUILD_EXAMPLES=ON \
  -DCOMOTION_BUILD_TESTS=ON
```

If nlohmann/json is not installed, CMake fetches it by default. Disable this
behavior with `-DCOMOTION_FETCH_NLOHMANN_JSON=OFF` in an offline or
package-manager-controlled environment.

The bundled CoMotion VAMP fork vendors `CPM.cmake` and may use CPM to fetch
`nigh`, `pdqsort`, and `SIMDxorshift` when local packages are unavailable.
Offline builds can provide those sources with
`-DCPM_nigh_SOURCE=/path/to/nigh-src`,
`-DCPM_pdqsort_SOURCE=/path/to/pdqsort-src`, and
`-DCPM_SIMDxorshift_SOURCE=/path/to/simdxorshift-src`.

### Run the Panda Cage Example

Run the built-in eight-robot Panda Cage scenario:

```bash
./build/apps/panda_cage --num-robots 8
```

This uses the standard Panda Cage parameters and ARC, the application's default
planner. Add `--output-paths` to write a viewer-compatible result, then follow
the [visualization instructions](#visualization).

## Library Usage

CoMotion provides the `comotion` CMake target for downstream C++ projects.
The minimal source-tree example links against it, constructs a one-robot
problem, and invokes the collision checker:

```bash
cmake --build build --target comotion_library_smoke
./build/examples/comotion_library_smoke
```

See [examples/library_smoke.cpp](examples/library_smoke.cpp) for the source and
[API.md](API.md) for the stable, baseline, experimental, and source-only API
surfaces.

To install CoMotion:

```bash
cmake --install build --prefix /path/to/comotion-install
```

The install includes the CoMotion CMake package, workload applications,
benchmark runners, and the Panda and Planar3 resources required by the public
benchmark cases. A downstream CMake project can consume the installed library
with:

```cmake
find_package(comotion REQUIRED)
target_link_libraries(my_target PRIVATE comotion::comotion)
```

The exported package carries the public dependencies on OMPL, Eigen3,
Boost.Serialization, Boost.Program_options, FCL, and nlohmann/json.

## Supported Examples and Applications

The standalone build provides:

- `comotion_library_smoke`: minimal downstream library and collision-checker
  example.
- `mobile_robot_2d_crossing`: flying-sphere parallel and circle scenarios.
- `planar_manipulator_cross`: crossing Planar3 manipulator scenarios.
- `panda_cage`: task-based multi-Panda cage scenarios.
- `panda_flat`: multi-Panda scenarios in a flat workspace.
- `heterogeneous_corridor`: mixed Panda and flying-sphere corridor scenarios.

Build individual application targets with:

```bash
cmake --build build --target mobile_robot_2d_crossing
cmake --build build --target planar_manipulator_cross
cmake --build build --target panda_cage
cmake --build build --target panda_flat
cmake --build build --target heterogeneous_corridor
```

Each application accepts `--help` for its supported scenarios, planners,
collision backends, and output options. Repeated experiments should use the
benchmark runners rather than invoking applications manually.

### Visualization

Applications write viewer-compatible result JSON with `--output-paths` or
`--output-endpoint-paths`. Serve the repository root and open the viewer:

```bash
python3 -m http.server 8000
```

Then visit `http://localhost:8000/viewer/` and load the generated
`*_result.json` file. ARC-family runs can also replay conflict detection and
repair history when both `--output-paths` and `--track-arc-history` are
supplied. See the [viewer documentation](viewer/README.md) for result loading,
ARC process playback, and controls.

## Benchmarks

The public benchmark runners execute applications from `build/apps` and
write results beneath `benchmarks/results/` by default:

```bash
# Preview the VA-MRMP planner/scenario/collision-backend trial matrix.
python3 benchmarks/scripts/run_planner_trials.py --dry-run

# Preview the final P-ARC paper matrix for the mobile and Planar3 scenarios.
python3 benchmarks/scripts/run_parallel_arc_2d.py --dry-run

# Preview the final P-ARC paper matrix for the Panda Cage scenario.
python3 benchmarks/scripts/run_parallel_arc_panda.py --dry-run

# Run feasibility comparisons and cumulative-success analysis over five seeds.
python3 benchmarks/scripts/run_feasibility.py --num-seeds 5

# Run anytime ARC, AO-ARC, and Composite AORRTC comparisons over five seeds.
python3 benchmarks/scripts/run_anytime.py --num-seeds 5

# Compare ARC and ParallelARC worker scaling over five seeds.
python3 benchmarks/scripts/run_multicore.py --num-seeds 5

# Plot collision-backend comparisons from an existing results file.
python3 benchmarks/scripts/plot_results.py path/to/results.csv --plot-backends
```


Each runner writes `manifest.json`, `results.csv`, `solution_events.csv`,
and its supported plots. See [BENCHMARKS.md](BENCHMARKS.md) for experiment
profiles, runner options, case catalogs, and the output contract.

## License

CoMotion is released under the BSD 3-Clause License; see [LICENSE](LICENSE).

Panda model assets and bundled header-only dependencies retain their upstream
licenses. Their provenance, local modifications, and corresponding license
texts are recorded in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) and
[LICENSES/](LICENSES/). Binary and source distributions install these notices
alongside the project documentation.

This repository bundles source checkouts of the CoMotion forks of OMPL and
VAMP. CoMotion OMPL remains under OMPL's BSD 3-Clause License in
`external/como-ompl/LICENSE`. CoMotion VAMP remains under VAMP's Apache
License 2.0 in `external/como-ompl/external/vamp/LICENSE.txt`, with additional
bundled third-party notices under
`external/como-ompl/external/vamp/licenses/`. Redistributions should
preserve those upstream license and notice files.
