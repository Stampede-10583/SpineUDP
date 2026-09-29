"""Rotating JSON Lines log for the Spine link.

One JSON object per line. Every record carries the run id, the wall time (UTC, ISO 8601) and the
Orin monotonic time, so records from one run can be grouped and ordered even across clock steps.
JSONL is for these offline files only; the link itself stays fixed-size binary.
"""

import json
import logging
import logging.handlers
import math
import os
import time
import uuid
from datetime import datetime, timezone


def json_safe(value):
    """Recursively replaces NaN and +/-inf (not valid JSON) with None."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


def new_run_id():
    return datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8]


class JsonlLog:
    """Writes records to <log_dir>/spine-orin.jsonl, rotating at max_bytes, keeping backup_count."""

    FILE_NAME = 'spine-orin.jsonl'

    def __init__(self, log_dir, max_bytes=10 * 1024 * 1024, backup_count=20, run_id=None):
        os.makedirs(log_dir, exist_ok=True)
        self.path = os.path.join(log_dir, self.FILE_NAME)
        self.run_id = run_id or new_run_id()
        self._logger = logging.getLogger(f'spine_stats.jsonl.{id(self)}')
        self._logger.setLevel(logging.INFO)
        self._logger.propagate = False
        self._handler = logging.handlers.RotatingFileHandler(
            self.path, maxBytes=max_bytes, backupCount=backup_count, encoding='utf-8')
        self._handler.setFormatter(logging.Formatter('%(message)s'))
        self._logger.addHandler(self._handler)

    def write(self, record_type, **fields):
        """Appends one record. Thread-safe (logging handlers lock internally)."""
        record = {
            'type': record_type,
            'run': self.run_id,
            'wall': datetime.now(timezone.utc).isoformat(timespec='milliseconds'),
            'mono': round(time.monotonic(), 6),
        }
        record.update(fields)
        self._logger.info(json.dumps(json_safe(record), separators=(',', ':'), sort_keys=False))

    def close(self):
        self._logger.removeHandler(self._handler)
        self._handler.close()
