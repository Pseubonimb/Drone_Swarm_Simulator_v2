"""MAVLINK_TELEMETRY_MODE parsing for MAVLinkWorker / utils."""

import os
import unittest
from unittest import mock

from core.mavlink.telemetry_mode import telemetry_uses_sim_state

_ENV = "MAVLINK_TELEMETRY_MODE"


class TelemetryModeTest(unittest.TestCase):
    def test_explicit_local(self) -> None:
        with mock.patch.dict(os.environ, {_ENV: "local"}, clear=False):
            self.assertFalse(telemetry_uses_sim_state())

    def test_sim_state_aliases(self) -> None:
        for val in ("sim_state", "SIM_STATE", "sim", "SimState"):
            with mock.patch.dict(os.environ, {_ENV: val}, clear=False):
                self.assertTrue(
                    telemetry_uses_sim_state(),
                    msg=f"expected sim mode for {val!r}",
                )

    def test_unset_defaults_sim_state(self) -> None:
        old = os.environ.pop(_ENV, None)
        try:
            self.assertTrue(telemetry_uses_sim_state())
        finally:
            if old is not None:
                os.environ[_ENV] = old


if __name__ == "__main__":
    unittest.main()
