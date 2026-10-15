# ZED RGB-D publisher source

The 20 original files in this directory were retrieved read-only from the
production publisher build source. Every file matches the archived source
manifest. The archived build result identifies the publisher binary with SHA-256
`2b1634ae850a340590257545e89f382b1d29625b494af318123fef6430fb9f50`,
which is also the binary selected by the captured mapping launcher.

Build on the target Linux environment with ROS 2 Humble, the modified
`rtabmap_msgs`, ZED SDK 4, its matching CUDA toolkit, OpenCV and CMake available:

```bash
cmake -S src/zed_rgbd_publisher -B build/zed_rgbd_publisher -DCMAKE_BUILD_TYPE=Release
cmake --build build/zed_rgbd_publisher --parallel 2
ctest --test-dir build/zed_rgbd_publisher --output-on-failure
```

The CMake project retains nine existing offline tests. Its archived build log
records nine passes; they were not rerun during this read-only export. No camera
or vehicle process was started and no field-validation claim is made. The CMake
project intentionally has no install rule. This source export contains neither
the proprietary ZED SDK nor a prebuilt publisher binary. Existing source notices
are preserved; no new license grant is inferred.
