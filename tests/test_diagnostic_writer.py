import importlib.util
import json
import tempfile
import threading
import unittest
from pathlib import Path

spec = importlib.util.spec_from_file_location('diagnostic_writer', Path(__file__).parents[1]/'tools/diagnostic_writer.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class WriterTests(unittest.TestCase):
    def test_immutable_records_drain(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'events.jsonl'
            writer = module.DiagnosticWriter(path)
            sample = {'value': 1}
            writer.append(sample)
            sample['value'] = 9
            writer.close()
            self.assertEqual(json.loads(path.read_text()), {'value': 1})
            with self.assertRaises(RuntimeError):
                writer.append(sample)

    def test_slow_sink_bounds_without_discarding_accepted_records(self):
        entered, release = threading.Event(), threading.Event()
        records = []
        def sink(text):
            entered.set()
            self.assertTrue(release.wait(2))
            records.append(json.loads(text))
        writer = module.DiagnosticWriter(None, capacity=1, sink=sink)
        try:
            writer.append({'n': 1})
            self.assertTrue(entered.wait(1))
            writer.append({'n': 2})
            with self.assertRaises(RuntimeError):
                writer.append({'n': 3})
        finally:
            release.set()
            writer.close()
        self.assertEqual(records, [{'n': 1}, {'n': 2}])

    def test_sink_failure_is_not_reported_as_complete(self):
        def sink(_):
            raise OSError('injected disk error')
        writer = module.DiagnosticWriter(None, sink=sink)
        writer.append({'n': 1})
        with self.assertRaises(RuntimeError):
            writer.close()
        with self.assertRaises(RuntimeError):
            writer.append({'n': 2})

    def test_byte_budget(self):
        writer = module.DiagnosticWriter(None, byte_limit=4, sink=lambda _: None)
        with self.assertRaises(RuntimeError):
            writer.append({'large': 'payload'})
        writer.close()


if __name__ == '__main__':
    unittest.main()
