# Candidate 8: boot-scoped light handoff on tmpfs

`field_v7_startup/` preserves the failed candidate 7 startup. It never opened the camera. The first observation of `startup_ready` was already 3.283 seconds old; the original two-second gate rejected it. This is observed state age, not a direct fsync duration measurement.

Candidate 8 keeps candidate 7's camera and candidate 5's worker. Only the light readiness runtime directory moves to verified, owned tmpfs. Identity, ownership, mode, unique controller, two-second freshness, and shutdown receipt checks remain. `package_result.json` records 37 Python regressions plus check-only. The controller code and motion behavior do not change.

Field v8 is a stationary RGB, cloud, capture and complete-save check. No new operator movement was requested and no movement keys were sent. It does not establish no-Tag relocalization acceptance. Production 8 remains unchanged.

`field_v8/` records the completed run: 5404 frames / 207.907 seconds / 196 segments, identical input and processing timestamps, full SVO verification, saved hashes, and a closed database quick check. All child and wrapper exits were zero. Source gaps stayed at or below 133.325 ms; no producer waits or post-initialization cuts/pauses occurred. Actual RViz screenshot and 30-second RGB/cloud topic probe are included. The stationary map has one keyframe; this does not validate moving recovery. Small logs and screenshots only, no full video/database copy.
