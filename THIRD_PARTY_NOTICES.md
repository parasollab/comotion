# Third-Party Notices

CoMotion itself is distributed under the BSD 3-Clause license in `LICENSE`.
The project also builds, installs, or redistributes the components and research
assets below. Their licenses remain applicable to those components.

## Panda Robot Resources

Files under `resources/panda/` are derived from the Panda resources distributed
by VAMP and Robowflex Resources:

- VAMP, Apache License 2.0: <https://github.com/KavrakiLab/vamp>
- Robowflex Resources, MIT License: <https://github.com/KavrakiLab/robowflex_resources>

The visual OBJ/MTL files and converted collision OBJ files are derived from
the Robowflex Panda resources distributed by VAMP. Robowflex Resources records
the upstream Panda sources as:

- `frankaemika/franka_ros` at
  `edba362bc216d7169f14801c92af70f4291a0f76` (Apache-2.0)
- `ros-planning/panda_moveit_config` at
  `27756a1f5a174f58b644e3670246b31a707ba828` (declared BSD in `package.xml`)

CoMotion packages the original visual meshes where available and uses the
simplified meshes for collision checking. The base and final arm links retain
their collision meshes as visuals because the source resource set does not
provide separate visual OBJ files for those links. No endorsement by Franka
Robotics, MoveIt, Rice University, or the Kavraki Lab is implied.

The applicable Apache-2.0, MoveIt BSD-3-Clause, and Robowflex Resources MIT
texts are included in `LICENSES/`.

## Bundled Planning Dependencies

- CoMotion's OMPL fork is derived from OMPL and remains under OMPL's BSD
  3-Clause license (`LICENSES/OMPL-BSD-3-Clause.txt`).
- VAMP is Apache-2.0 (`LICENSES/Apache-2.0.txt`).
- nlohmann/json is MIT (`LICENSES/nlohmann-json-MIT.txt`).
- nigh, pdqsort, and SIMDxorshift retain the license texts named for them in
  `LICENSES/`.

System-provided dependencies such as Boost, Eigen, and FCL are not copied into
the CoMotion source tree; consult those projects for their respective terms.
