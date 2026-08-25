# Resources

This directory contains the robot descriptions used by the public CoMotion apps
and benchmark workloads.

Included resource families:

- Panda
- Planar3

The `*_spherized.urdf` files provide conservative sphere decompositions used by
the sphere and VAMP collision backends. The Panda mesh URDF and collision meshes
support the FCL benchmark backend. The public source tree intentionally ships
only the Panda and Planar3 models required by the benchmark runners.

Large unused robot families, problem archives, demo media, heightfields, and
legacy conversion helpers were removed from the public source tree. The public
CoMotion benchmark entry points are the three runners documented in
[../BENCHMARKS.md](../BENCHMARKS.md).

## Provenance And Release Status

The table below records the release status visible from this repository.

| Resource | Contents | Release status |
| --- | --- | --- |
| `planar3/` | Synthetic planar manipulator URDF, SRDF, and sphere model | Local synthetic model. |
| `panda/` | Panda URDF/SRDF, conservative sphere model, and visual/collision meshes | Derived from the VAMP/Robowflex Panda resources. See `THIRD_PARTY_NOTICES.md` and `LICENSES/` at the repository root. The packaged URDF uses the original visual meshes where available and retains the simplified meshes for collision checking. |
