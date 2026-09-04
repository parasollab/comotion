# CoMotion API Surface

CoMotion installs a deliberately small public header set for downstream C++
projects. Headers that remain in the source tree but are not installed are
implementation details for the library, apps, benchmarks, or regression tests.

## Stable API

These interfaces are intended as the primary downstream integration surface for
CoMotion 0.1.x:

- Problem and planner scaffolding: `MultiRobotProblem`, `MultiRobotPlanner`,
  `Path`
- Collision and robot modeling: `CollisionChecker`, `ConflictChecker`,
  `RobotModel`, `FlyingSphere`
- Seeding helpers: `PlanningRng`, `PlanningSeed`
- Primary planners: `ARC`, `AOARC`

`AOARC` reuses incumbent paths that already satisfy its next discrete reuse
threshold and skips the initial conflict scan for unchanged/unchanged robot
pairs by default. These optimizations are independently configurable with
`setSelectiveBoundedReplanning(bool)` and
`setSelectiveInitialConflictScan(bool)`. The inclusive reuse threshold is
`B - max(epsilon, 1 timestep)`, with saturating subtraction; replanned paths
continue to use Bounded ARC's global bound `B`.
`setRepairHistoryReplanningDepth(size_t)` additionally replans robots connected
to a bound violator through accepted incumbent repairs: depth 0 is
violators-only, depth 1 adds direct repair partners, and higher depths expand
breadth-first. `setRandomFullRestartProbability(double)` independently chooses
a deterministic, seed-replayable full restart for each bounded call, allowing
selective AO-ARC to escape an incumbent path basin. If that restart improves
the incumbent, its repair history replaces the discarded paths' history;
selective improvements continue to append history. The probability defaults
to zero. The
former `setExpandReplanningFromRepairHistory(bool)` API remains as a depth 0/1
compatibility wrapper.

## Baseline API

These planners are installed for comparisons, evaluator baselines, and small
problem instances:

- `CompositeRRT`
- `CompositeRRTStar`
- `CompositePRMStar`
- `PrioritizedSTRRT`
- `MRdRRT`
- `MRdRRTStar`

## Experimental And Advanced API

These interfaces are installed, but their details may change more readily than
the stable API:

- `ParallelARC`
- `CompositeAORRTC`
- `STCBS`
- `USTRRTstar`
- `AORRTCUtils`

`USTRRTstar` is installed because `STCBS` exposes `USTRRTstar::RewireMode` in
its public configuration API.

`ParallelARC` also exposes the experimental conflict-find assignment modes
declared in `ValidationTypes.h`. The public P-ARC reproduction profile selects
`CyclicCoverGreedy`; the other modes remain available for the paper ablations.

## Source-Only Internals

The following headers are intentionally not installed:

- `OrParallelPlanner`
- `CooperativeCompositeRRT`
- `MakespanCompositeStateSpace`
- `MakespanInformedSampler`
- Any header under a `detail/` directory

Source-only internals may still be used by CoMotion's own apps, benchmarks, and
tests, but downstream projects should not include them from an installed
package.
