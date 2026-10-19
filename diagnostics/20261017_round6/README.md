# Round-six candidate evidence (physical recovery acceptance pending)

- `first_trial_black_window.png`: real first trial, `custom_20261010_064722`; topics existed but the RViz rendering areas were black. This is a failed display test, regardless of the earlier topic-only script's `passed=true`.
- `render_retry_real.log`: real next camera launch, `custom_20261010_065941`. First attempt produced three `GLXBadDrawable` errors. The bounded initialization retry then reached `WBROV_RVIZ_RENDER_READY` on attempt 1, without restarting the mapping or camera processes.
- `render_retry_visible.png`: real screenshot from the second trial, confirming image panels, grid, and main point cloud rendered after that retry.
- `render_guard_tests.json`: separate fault-injection executable tests bounded retry/exit; production binary contains no injection code.
- `render_admission_tests.json`: actual launcher's admission function tested with ready, black, dead/changed display identity, and failed sibling conditions.

- `stationary_trial_closed.json`: final closed-run audit for the second trial. Rendering succeeded, the launcher exited normally with code 0, and save/export/cleanup completed. The saved main cloud contains 901 vertices; the closed recording index contains 30,692 frames, with boundary frames decoded. This does not certify full recording replay or sensor-level losslessness.

The second trial ended through the bounded test's normal RViz-close/save path before the follow-up stop request. The light owner sent six zero-brightness packets and exited; UDP3333 is free and no owned trial processes remain. There is no physical lamp acknowledgement. Production launcher hashes remain unchanged.

The operator was unavailable for movement, so automatic reference accumulation under movement and occlusion recovery were not exercised. The main graph stayed at one node, epoch 0, cut count 0. Freshness pauses occurred and recovered; final state was NORMAL. Status counts are samples, not distinct pause events.

The generic stationary probe is also **false**: isolated odometry age reached 575.437 ms, exceeding its 500 ms maximum-age threshold (P95 126.447 ms). Do not turn this into a passed result or relax the threshold to obtain acceptance. Retain it for latency investigation before the next trial.

Closed-log follow-up (`stationary_latency.json`, `stationary_latency_log_context.json`) narrows the latency issue without restarting hardware:

- `/odom`: 22,660 samples, six above 300 ms and two above 500 ms; P99 137.725 ms. The independent subscriber and the odometry process agree on the worst delay (575.437 / 575.291 ms), so this is not solely the image/cloud probe's callback load.
- The corresponding odometry registration took 20.010 ms, with 182 inliers and no lost flag. This sample does not show a slow geometric registration.
- During the same stall, the camera's `CAPTURE_SLOW_PATH` reports `plan_log_ms=483.302`, only 0.226 ms of CPU in its pre-grab span, and 39.471 ms inside grab. This identifies a long wait in the synchronous plan-log call on the capture path. It does not establish whether the underlying cause was logging infrastructure, storage, or scheduling, nor that it caused every delayed odometry sample.
- The next measured interval was 516.634 ms, with a 1.577 mm / 0.088435 degree step. Existing measured-gap validation preserved the chain; the final cut count stayed zero. This was not a manufactured occlusion/relocalization test.

Next independent investigation: remove or isolate capture-path synchronous diagnostic logging with bounded output behavior, account for dropped diagnostics explicitly, then remeasure. Do not change loss/recovery thresholds on the strength of this stationary log.

This directory does not certify loss recovery, continued map growth, deployment, or underwater accuracy. No propulsion or unlock was requested in either camera trial. No production 8 launcher or GitHub branch was updated for this candidate.
