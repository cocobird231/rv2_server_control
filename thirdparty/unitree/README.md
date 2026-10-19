# Unitree ROS interface sources

These sources come from [unitreerobotics/unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)
commit `5204e6e098ee53f4bd929bd77eb1d387cd0fa842`, directory
`cyclonedds_ws/src/unitree/{unitree_api,unitree_go,unitree_hg}`.
The upstream BSD 3-Clause license is preserved in [LICENSE](LICENSE).

All 43 message definitions and the upstream CMake files are unchanged.
The only package patch adds the missing `rosidl_generator_dds_idl` build dependency
to each `package.xml`, matching its existing CMake requirement.

The server depends on `unitree_api` for sport requests and `unitree_go` for
WirelessController. Its build helper resolves each independently from workspace
sources, the sourced underlay, then the embedded package directory.
`test_depends.repos` mounts both embedded sources explicitly
for reproducible tests; it does not use an external Unitree checkout.

The parent `AMENT_IGNORE` keeps upstream files outside the server's recursive
ament lint checks. Select a package directory directly to discover/build it, for
example `--base-paths thirdparty/unitree/unitree_api`; do not place an ignore marker
inside the package itself. `unitree_hg` is retained as supplied,
but is not selected by the server's build helper or test manifest.
