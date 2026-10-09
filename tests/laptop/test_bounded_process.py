from __future__ import annotations

import os
import time
import unittest

from laptop.diagnostics.bounded_process import run_bounded


def _return_one() -> dict:
    return {"value": 1}


def _block_forever() -> None:
    time.sleep(60)


def _exit_without_result() -> None:
    os._exit(17)


class BoundedProcessTest(unittest.TestCase):
    def test_normal_worker_returns(self) -> None:
        report = run_bounded(_return_one, timeout_s=5)
        self.assertEqual(report["status"], "COMPLETE")
        self.assertEqual(report["result"], {"value": 1})

    def test_blocked_worker_is_terminated(self) -> None:
        start = time.monotonic()
        report = run_bounded(_block_forever, timeout_s=0.4)
        self.assertEqual(report["status"], "TIMEOUT_TERMINATED")
        self.assertLess(time.monotonic() - start, 4)

    def test_worker_process_exit_without_result_fails_closed(self) -> None:
        report = run_bounded(_exit_without_result, timeout_s=5)
        self.assertEqual(report["status"], "WORKER_ERROR")
        self.assertEqual(report["error"], "worker exited without result")
        self.assertEqual(report["worker_exitcode"], 17)


if __name__ == "__main__":
    unittest.main()
