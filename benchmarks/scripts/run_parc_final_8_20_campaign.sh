#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
output_root="${1:-${repo_root}/benchmarks/results/parc_final_8_20}"
build_root="${repo_root}/build"
build_apps="${build_root}/apps"

mkdir -p "${output_root}"
export COMOTION_VALIDATION_INSTRUMENTATION=0

run_phase() {
    local phase_name="$1"
    shift
    printf '[%s] START %s\n' "$(date --iso-8601=seconds)" "${phase_name}"
    "$@"
    printf '[%s] COMPLETE %s\n' "$(date --iso-8601=seconds)" "${phase_name}"
}

run_phase build \
    cmake --build "${build_root}" --target \
    mobile_robot_2d_crossing planar_manipulator_cross panda_cage -j 16

run_phase 2d_scaling_and_baselines \
    python3 "${repo_root}/benchmarks/scripts/run_parc_final_2d.py" \
    --output-root "${output_root}" --build-dir "${build_apps}" \
    --max-cores 16 --skip-build

run_phase panda_baselines \
    python3 "${repo_root}/benchmarks/scripts/run_final_panda_parallel_arc.py" \
    --output-root "${output_root}/panda" --build-dir "${build_apps}" \
    --skip-build

run_phase independence_ablation \
    python3 "${repo_root}/benchmarks/scripts/run_parallel_arc_optimistic_ablation.py" \
    --output-root "${output_root}/independence_ablation" \
    --build-dir "${build_apps}" --jobs 1

run_phase synchronization_horizon_ablation \
    python3 "${repo_root}/benchmarks/scripts/run_parallel_arc_conflict_ablation.py" \
    --output-root "${output_root}/synchronization_horizon_ablation" \
    --build-dir "${build_apps}" --jobs 1

printf '[%s] CAMPAIGN COMPLETE: %s\n' \
    "$(date --iso-8601=seconds)" "${output_root}"
