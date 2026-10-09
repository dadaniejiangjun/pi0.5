from __future__ import annotations

import unittest

from server.training.proxy_latency_profile import profile


class ProxyLatencyProfileTest(unittest.TestCase):
    def test_bounded_settings_reject_zero_measurements_before_loading_model(self) -> None:
        for settings in ({"samples": 0}, {"warmup": 0}, {"num_steps": 1},
                         {"num_steps": 11}, {"samples": 21}):
            with self.subTest(settings=settings), self.assertRaisesRegex(ValueError, "bounded"):
                profile(**settings)


if __name__ == "__main__":
    unittest.main()
