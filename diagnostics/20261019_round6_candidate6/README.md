# Candidate 6: capture drain ownership

`field_v5/` preserves candidate 5's failed near-distance field run, including the old timed shutdown that killed capture with pending RAM segments. All files in its `sha256.json` are exact raw bytes. No original video/database backup is included.

`package_result.json` records 36 passing Python regressions and package check-only. `delayed_drain_result.json` records a real child waiting 75 seconds after SIGINT (80.97 seconds observed), then writing a completion sentinel and exiting normally. Its count of 100 is a synthetic sentinel, not 100 RGB-D frames. This is not a camera or robot test.

`round6_field_notags_v6.py` uses an explicit stop file; there is no 300-second capture limit. Startup readiness still has a bound. It does not send movement keys or change production 8. `field_v6/` preserves its failure: the independent SVO journal overflowed at 512 queued records. All 2882 RGB-D frames were drained and processed; saved hashes and the closed database passed. The SVO counter was one frame ahead, so this is interrupted salvage, not complete recording acceptance. After the first six initialization frames, all recorded frames allowed main-map writes with no new cut.

Candidate 6 keeps candidate 5's camera/worker binaries and quality/input rules. Only the session's capture-exit timeout and the associated tests change. Production deployment and full field recovery acceptance are still pending.
