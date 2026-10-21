"""Nonblocking request to the existing ROS home service; never owns CAN."""
import subprocess
import time
from pathlib import Path


class HomeBeforeStart:
    def __init__(self, command, log_dir, *, popen=subprocess.Popen, clock=time.monotonic):
        self.command = command
        self.log_dir = Path(log_dir)
        self.popen = popen
        self.clock = clock
        self.process = None
        self.log = None
        self.path = None
        self.started = None

    @property
    def busy(self):
        return self.process is not None

    def start(self):
        if self.busy:
            raise RuntimeError('home is already running; no duplicate request sent')
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.path = self.log_dir / f'home_{time.time_ns()}.log'
        self.log = self.path.open('x', encoding='utf-8')
        try:
            self.process = self.popen(self.command, stdin=subprocess.DEVNULL,
                                      stdout=self.log, stderr=subprocess.STDOUT)
        except Exception:
            self.log.close()
            raise
        self.started = self.clock()

    def poll(self):
        if self.process is None:
            return None
        code = self.process.poll()
        if code is None:
            if self.clock() - self.started > 115:
                raise RuntimeError(f'home confirmation timeout; recording blocked; log={self.path}')
            return None
        self.log.close()
        elapsed = self.clock() - self.started
        self.process = None
        return {'ok': code == 0, 'exit_code': code, 'elapsed_s': round(elapsed, 3),
                'log': str(self.path)}

    def close(self):
        """Release our log handle; never cancel or claim to stop a home request."""
        running = self.process is not None and self.process.poll() is None
        if self.log is not None:
            self.log.close()
        return running
