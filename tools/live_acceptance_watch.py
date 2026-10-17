"""Bound a read-only probe by the lifetime of the mapping session it observes."""
from pathlib import Path
import subprocess
import time


def process_alive(pid, start_ticks):
    """Check Linux process identity as well as PID; zombies are already stopped."""
    try:
        fields = (Path('/proc') / str(pid) / 'stat').read_text().rsplit(')', 1)[1].split()
        return fields[19] == str(start_ticks) and fields[0] not in ('Z', 'X')
    except (OSError, IndexError):
        return False


def wait_for_probe(probe, launcher, rviz_alive, *, timeout, heartbeat=None):
    """Fail promptly on session exit, even if the passive probe is still alive.

    The caller owns all three processes. Only the probe is terminated here;
    launcher shutdown and map saving remain the caller's responsibility.
    """
    deadline = time.monotonic() + timeout
    try:
        while True:
            code = launcher.poll()
            if code is not None:
                raise RuntimeError('Mapping launcher exited during probe: ' + str(code))
            if not rviz_alive():
                raise RuntimeError('Owned RViz exited during probe')
            code = probe.poll()
            if code is not None:
                return code
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('Live probe exceeded its bounded observation time')
            if heartbeat is not None:
                heartbeat(remaining)
            time.sleep(min(.2, remaining))
    finally:
        if probe.poll() is None:
            probe.terminate()
            try:
                probe.wait(timeout=2)
            except subprocess.TimeoutExpired:
                probe.kill()
                probe.wait(timeout=2)
