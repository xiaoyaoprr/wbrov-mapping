# Native mapping helpers

The two `*_native` directories preserve the source, headers, fixture programs and
CMake definitions recovered from the production build directory. The existing
remote build outputs match the corresponding hashes in the current runtime
manifest. No fixture, ROS node or robot process was launched during this export.

Build each directory separately on the target Linux/ROS 2 environment after
sourcing ROS 2 Humble and the workspace containing this package's modified
`rtabmap_msgs`:

```bash
cmake -S src/native_helpers/header_health_observer_native -B build/header_health -DCMAKE_BUILD_TYPE=Release
cmake --build build/header_health --parallel 2
cmake -S src/native_helpers/safe_reloc_gid_frontend_native -B build/safe_reloc_gid -DCMAKE_BUILD_TYPE=Release
cmake --build build/safe_reloc_gid --parallel 2
cmake -S src/native_helpers/message_info -B build/message_info -DCMAKE_BUILD_TYPE=Release
cmake --build build/message_info --parallel 2
```

`safe_reloc_gid_frontend` additionally requires OpenSSL development headers.
The message-info adapter uses `rcl`, `rcutils` and `rmw`. Its CMake definition was
added for portability during packaging; its original source and archived build
receipt were preserved, but this new definition was not compiled in this export.
The adapter exposes the serialized message and publisher GID from the same
receive operation.

The fixtures publish synthetic ROS messages and are intended for an isolated
test environment. Building these programs does not authorize running them on a
live mapping ROS domain. This source export contains no prebuilt native binaries.
Existing source notices are preserved; no new license grant is inferred.
