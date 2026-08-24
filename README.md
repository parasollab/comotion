# CoMotion

CoMotion, short for Coordinated Motion Planning for Multi-Robot Systems, is a
C++ multi-robot motion planning library and benchmark suite built on the
CoMotion forks of OMPL and VAMP.

The repository has two public entry points:

- `comotion`, the current CMake library target for downstream C++ projects
- standalone workload executables in `apps/`, driven directly or through the
  benchmark runners in `benchmarks/scripts/`

## Repository Layout

```text
src/                  CoMotion library sources and public headers
apps/                 standalone workload executables
benchmarks/scripts/   feasibility, anytime, and multi-core runners
benchmarks/tests/     benchmark geometry/regression tests
examples/             minimal downstream library-use examples
resources/            minimal robot models and benchmark resources
viewer/               optional path-result visualization tool
tests/                library and planner regression tests
external/como-ompl    CoMotion OMPL fork, including the CoMotion VAMP fork
```

## Public API

The installed C++ headers are intentionally smaller than the source tree. See
[API.md](API.md) for the stable, baseline, experimental, and source-only
interfaces.

## Prerequisites

CoMotion currently targets Linux-style C++ research environments. Before
configuring the project, make sure these dependencies are available:

- C++17 compiler and CMake 3.14 or newer: compile the CoMotion library, the
  bundled CoMotion OMPL fork, and the benchmark apps.
- Git with submodule support: fetch `external/como-ompl`, its CoMotion VAMP
  submodule, and nested third-party submodules.
- Boost.Serialization and Boost.Program_options: required by OMPL and by
  CoMotion planner/app configuration paths.
- Eigen3: used by robot kinematics, geometric state operations, OMPL headers,
  and VAMP integration.
- FCL and `pkg-config`: provide the optional FCL collision backend and the CMake
  discovery path used by CoMotion.
- nlohmann/json 3.11.3 or newer: used for app inputs, metrics JSON, benchmark
  manifests, and planner statistics. CMake fetches it by default when it is not
  installed.
- Python 3.10 or newer: run the benchmark scripts in `benchmarks/scripts/`.
  The scripts use only the standard library for trial execution and CSV/JSON
  output.
- matplotlib, optional: generate benchmark PNG/SVG plots. Without matplotlib,
  the benchmark runners still write `manifest.json`, `results.csv`, and
  `solution_events.csv`.

ParallelARC and the multi-core benchmark path use POSIX process primitives
(`fork`, pipes, signals), so Linux is the expected platform for those workloads.
VAMP chooses architecture-specific SIMD flags by default; use `VAMP_ARCH` if
you need portable binaries for a different CPU target.

## Build

```bash
git submodule update --init --recursive
cmake -S . -B build
cmake --build build
```

CoMotion builds the bundled CoMotion OMPL fork from `external/como-ompl`; that
fork builds the bundled CoMotion VAMP fork from
`external/como-ompl/external/vamp`.

Useful build options:

```bash
cmake -S . -B build \
  -DCOMOTION_BUILD_APPS=ON \
  -DCOMOTION_BUILD_EXAMPLES=ON \
  -DCOMOTION_BUILD_TESTS=ON
```

If `nlohmann_json` is not installed, CMake fetches it by default. Disable that
with `-DCOMOTION_FETCH_NLOHMANN_JSON=OFF` when building in an offline/package-manager
environment.

The bundled CoMotion VAMP fork vendors `CPM.cmake` and may use CPM to fetch
`nigh`, `pdqsort`, and `SIMDxorshift` when local packages are not available.
Offline builds can preseed those dependency sources with CMake variables such
as `-DCPM_nigh_SOURCE=/path/to/nigh-src`,
`-DCPM_pdqsort_SOURCE=/path/to/pdqsort-src`, and
`-DCPM_SIMDxorshift_SOURCE=/path/to/simdxorshift-src`.

The installed CMake package exports the dependencies used by CoMotion public
headers and targets: OMPL, Eigen3, Boost.Serialization, Boost.Program_options,
FCL, and nlohmann/json.

## Library Example

```bash
cmake --build build --target comotion_library_smoke
./build/examples/comotion_library_smoke
```

The source is [examples/library_smoke.cpp](examples/library_smoke.cpp). It links
against `comotion`, constructs a one-robot problem, and calls the collision checker.

## Install And Link From Another Project

```bash
cmake --install build --prefix /path/to/comotion-install
```

The install includes the CoMotion CMake package, currently exported as
`comotion`, workload apps in `bin/`, benchmark scripts under
`share/comotion/benchmarks/scripts/`, and the minimal Panda and Planar3 resources
needed by the public benchmark cases.

Downstream CMake projects should consume the installed package through
`find_package`:

```cmake
find_package(comotion REQUIRED)
target_link_libraries(my_target PRIVATE comotion::comotion)
```

## Standalone Workloads

The workload applications build under `build/apps/`:

```bash
cmake --build build --target mobile_robot_2d_crossing
cmake --build build --target planar_manipulator_cross
cmake --build build --target panda_cage
cmake --build build --target panda_flat
cmake --build build --target heterogeneous_corridor
```

They can be run directly, but repeated experiments should use the public
benchmark runners.

## Benchmarks

Common benchmark entry points:

```bash
python3 benchmarks/scripts/run_parallel_arc_2d.py --dry-run
python3 benchmarks/scripts/run_parallel_arc_panda.py --dry-run
python3 benchmarks/scripts/run_feasibility.py --num-seeds 5
python3 benchmarks/scripts/run_anytime.py --num-seeds 5
python3 benchmarks/scripts/run_multicore.py --num-seeds 5
python3 benchmarks/scripts/run_planner_trials.py --dry-run
python3 benchmarks/scripts/plot_results.py path/to/results.csv --plot-backends
```

The two Parallel ARC reproduction runners use the final paper parameters as
the standard profiles for the Mobile, Planar Cross, and Panda Cage benchmarks.
They pin the complete profiles in each generated experiment manifest even
though the corresponding workload executables use the same defaults.

Each runner writes `manifest.json`, `results.csv`, `solution_events.csv`,
and the supported plots. See [BENCHMARKS.md](BENCHMARKS.md) for the output
contract and case catalog.

## Visualization

The optional browser viewer can load path-result JSON files produced by direct
app runs with `--output-paths` or `--output-endpoint-paths`. See
[viewer/README.md](viewer/README.md).

## Citation

If you use CoMotion, please cite:

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
bundled third-party notices under `external/como-ompl/external/vamp/licenses/`.
Redistributions should preserve those upstream license and notice files.
