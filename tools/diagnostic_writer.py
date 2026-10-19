"""Bounded diagnostic output: callbacks enqueue immutable text, never write files."""
import json
import queue
import threading


class DiagnosticWriter:
    def __init__(self, path, capacity=1024, byte_limit=8 * 1024 * 1024, sink=None):
        self._queue = queue.Queue(maxsize=capacity)
        self._limit = byte_limit
        self._bytes = 0
        self._lock = threading.Lock()
        self._closing = threading.Event()
        self._ready = threading.Event()
        self._error = None
        self._thread = threading.Thread(target=self._run, args=(path, sink), daemon=True)
        self._thread.start()
        if not self._ready.wait(5):
            raise RuntimeError('diagnostic writer did not open its sink')
        self._check()

    def _check(self):
        if self._error is not None:
            raise RuntimeError('diagnostic writer failed; measurements are incomplete') from self._error

    def append(self, record):
        text = json.dumps(record, separators=(',', ':'), allow_nan=False) + '\n'
        size = len(text.encode('utf-8'))
        with self._lock:
            self._check()
            if self._closing.is_set():
                raise RuntimeError('diagnostic writer is closing')
            if self._bytes + size > self._limit:
                raise RuntimeError('diagnostic writer byte budget exhausted')
            try:
                self._queue.put_nowait((text, size))
            except queue.Full as error:
                raise RuntimeError('diagnostic writer record budget exhausted') from error
            self._bytes += size

    def _run(self, path, sink):
        stream = None
        try:
            if sink is None:
                stream = open(path, 'x', encoding='utf-8')
                sink = stream.write
            self._ready.set()
            while not (self._closing.is_set() and self._queue.empty()):
                try:
                    text, size = self._queue.get(timeout=.05)
                except queue.Empty:
                    continue
                sink(text)
                with self._lock:
                    self._bytes -= size
        except BaseException as error:
            self._error = error
        finally:
            try:
                if stream is not None:
                    stream.close()
            except BaseException as error:
                self._error = error
            self._ready.set()

    def close(self):
        with self._lock:
            self._closing.set()
        self._thread.join(timeout=10)
        if self._thread.is_alive():
            raise RuntimeError('diagnostic sink did not drain; measurements are incomplete')
        self._check()

