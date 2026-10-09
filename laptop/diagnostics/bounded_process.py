"""Spawn a disposable worker so blocking device drivers cannot trap the CLI."""

from __future__ import annotations

import multiprocessing as mp
import time
from typing import Callable


def _entry(sender, target: Callable, args: tuple, kwargs: dict) -> None:
    try:
        sender.send(("ok", target(*args, **kwargs)))
    except BaseException as exc:
        sender.send(("error", f"{type(exc).__name__}: {exc}"))
    finally:
        sender.close()


def run_bounded(target: Callable, *, args: tuple = (), kwargs: dict | None = None,
                timeout_s: float) -> dict:
    """Return a result or terminate the entire disposable worker process."""
    if not 0 < timeout_s <= 180:
        raise ValueError("timeout_s must be in (0, 180]")
    context = mp.get_context("spawn")
    receiver, sender = context.Pipe(duplex=False)
    worker = context.Process(target=_entry, args=(sender, target, args, kwargs or {}))
    started = time.monotonic()
    try:
        worker.start()
        sender.close()
        remaining = max(0.0, timeout_s - (time.monotonic() - started))
        if receiver.poll(remaining):
            try:
                kind, value = receiver.recv()
            except EOFError:
                kind, value = "error", "worker exited without result"
            worker.join(timeout=1)
            if worker.is_alive():
                worker.terminate()
                worker.join(timeout=1)
                return {"status": "WORKER_ERROR", "result": None,
                        "error": "worker reported completion but did not exit",
                        "worker_exitcode": worker.exitcode}
            return {"status": "COMPLETE" if kind == "ok" else "WORKER_ERROR",
                    "result": value if kind == "ok" else None,
                    "error": value if kind != "ok" else None,
                    "worker_exitcode": worker.exitcode}
        worker.terminate()
        worker.join(timeout=1)
        if worker.is_alive():
            worker.kill()
            worker.join(timeout=1)
        return {"status": "TIMEOUT_TERMINATED" if not worker.is_alive() else "TERMINATION_UNCONFIRMED",
                "result": None, "error": "capture worker exceeded deadline",
                "worker_exitcode": worker.exitcode}
    finally:
        receiver.close()
        if worker.is_alive():
            worker.kill()
            worker.join(timeout=1)
