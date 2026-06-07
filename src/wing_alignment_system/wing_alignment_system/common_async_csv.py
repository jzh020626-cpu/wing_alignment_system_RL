# -*- coding: utf-8 -*-

import csv
import os
import queue
import threading
import time
from typing import List


class AsyncCsvLogger:
    def __init__(self, path: str, fieldnames: List[str]):
        self.path = path
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self._q = queue.Queue(maxsize=20000)
        self._stop = threading.Event()
        self._fieldnames = list(fieldnames)

        self._fp = open(path, "w", newline="", encoding="utf-8")
        self._wr = csv.DictWriter(self._fp, fieldnames=self._fieldnames, extrasaction='ignore')
        self._wr.writeheader()
        self._fp.flush()

        self._thr = threading.Thread(target=self._run, daemon=True)
        self._thr.start()

    def log(self, row: dict):
        try:
            self._q.put_nowait(row)
        except queue.Full:
            pass

    def close(self):
        self._stop.set()
        self._thr.join(timeout=1.0)

        while not self._q.empty():
            try:
                self._wr.writerow(self._q.get_nowait())
            except queue.Empty:
                break

        self._fp.flush()
        self._fp.close()

    def _run(self):
        last_flush = time.time()
        while not self._stop.is_set():
            try:
                row = self._q.get(timeout=0.2)
                self._wr.writerow(row)
            except queue.Empty:
                pass

            if time.time() - last_flush >= 0.5:
                self._fp.flush()
                last_flush = time.time()
