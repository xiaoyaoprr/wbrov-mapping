# Private RViz entry point

This entry point keeps the upstream BSD-3-Clause copyright notice and uses the installed ROS 2 RViz libraries. It calls `rviz_rendering::RenderSystem::forceNoStereo()` before initializing `VisualizerApp`. It does not replace system RViz, Ogre, or graphics drivers.

Build on the target ROS 2 installation:

```sh
. /opt/ros/humble/setup.sh
cmake -S . -B build
cmake --build build --parallel 1
```

The Jetson build used rviz_common 11.2.26 and rclcpp 16.0.18. Its executable SHA-256 was `7537d5e715fc2ec0ce5a7acb4c44757671eb70c9de4db2fc21e527442549908d`; a rebuild on another system is not expected to have an identical binary hash.

Three isolated launches of each entry point passed. The original entry point did not reproduce the intermittent black window in that comparison, so this is not proof of the original GLX failure's cause. Subsequent candidate runs displayed real camera images and point clouds; mapping recovery and saving are separately tested.

## Startup rendering guard (round six candidate)

The first round-six camera trial reproduced `GLXBadDrawable` with black panels despite valid image and point-cloud topics. Disabling stereo alone did not prevent it. Six subsequent isolated probes (including native Qt, no anti-aliasing, EGL, and no Qt GL integration) did not reproduce it; they do not establish a graphics-driver cause or justify changing the system renderer.

The candidate captures context-creation errors during initial window setup. Two seconds after initialization, it either logs `WBROV_RVIZ_RENDER_READY`, or shuts down Qt/ROS and re-executes the same executable in the same PID. At most two retries are allowed. A third failure exits with status 75. Mapping and camera processes are not restarted. There is no retry of a later user close or later mapping failure. The launcher waits for this startup receipt rather than treating an alive black window as ready. Actual screenshot inspection remains part of acceptance; the receipt alone is not pixel-level proof.

The separate fault-injection fixture passed both two-failures-then-success and persistent-failure cases. Injection code is not present in the production executable. Real camera, point-cloud rendering, recovery, and saving still require the candidate's next hardware trial.

Upstream context: [Jetson report](https://github.com/ros2/rviz/issues/1314) and [RViz rendering initialization](https://github.com/ros2/rviz/blob/humble/rviz_rendering/src/rviz_rendering/render_system.cpp). These are background references, not proof of the device-specific cause.
