"""A running probe must not conceal a launcher or RViz exit."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import time
import unittest

spec = importlib.util.spec_from_file_location('live_acceptance_watch',
    Path(__file__).resolve().parents[1] / 'tools/live_acceptance_watch.py')
watch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(watch)


class SessionWatchTests(unittest.TestCase):
    def spawn(self, source):
        child = subprocess.Popen([sys.executable, '-B', '-c', source],
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        def cleanup():
            if child.poll() is None:
                child.terminate()
            child.wait(timeout=5)
        self.addCleanup(cleanup)
        return child

    def test_launcher_exit_stops_long_probe_promptly(self):
        launcher = self.spawn('import time; time.sleep(.1); raise SystemExit(7)')
        probe = self.spawn('import time; time.sleep(30)')
        started = time.monotonic()
        with self.assertRaisesRegex(RuntimeError, 'launcher exited.*7'):
            watch.wait_for_probe(probe, launcher, lambda: True, timeout=10)
        self.assertLess(time.monotonic() - started, 3)
        self.assertIsNotNone(probe.poll())

    def test_rviz_exit_stops_probe_leaving_launcher_to_save(self):
        launcher = self.spawn('import time; time.sleep(30)')
        probe = self.spawn('import time; time.sleep(30)')
        with self.assertRaisesRegex(RuntimeError, 'RViz exited'):
            watch.wait_for_probe(probe, launcher, lambda: False, timeout=10)
        self.assertIsNotNone(probe.poll())
        self.assertIsNone(launcher.poll())

    def test_completed_probe_cannot_hide_already_exited_launcher(self):
        launcher = self.spawn('raise SystemExit(1)')
        probe = self.spawn('pass')
        launcher.wait(timeout=5); probe.wait(timeout=5)
        with self.assertRaisesRegex(RuntimeError, 'launcher exited'):
            watch.wait_for_probe(probe, launcher, lambda: True, timeout=10)

    def test_normal_completion_and_bounded_timeout(self):
        launcher = self.spawn('import time; time.sleep(30)')
        probe = self.spawn('pass')
        self.assertEqual(watch.wait_for_probe(probe, launcher, lambda: True, timeout=5), 0)
        blocked = self.spawn('import time; time.sleep(30)')
        with self.assertRaises(TimeoutError):
            watch.wait_for_probe(blocked, launcher, lambda: True, timeout=.1)
        self.assertIsNotNone(blocked.poll())
        self.assertIsNone(launcher.poll())


if __name__ == '__main__':
    unittest.main()
