"""Four-wheel command path through the copied DDSM HAT driver, without hardware."""

import json
import time
import unittest
from unittest.mock import patch

from crsf import CRSFSnapshot
from ddsm115 import DDSMHat
from robot_main import Robot, drive_request
from test_robot_main import settings, snapshot


class SimulatedHatSerial:
    def __init__(self):
        self.timeout = 0.01
        self.write_timeout = 0.15
        self.closed = False
        self.buffer = bytearray()
        self.writes = []
        self.modes = {mid: 2 for mid in (1, 2, 3, 4)}
        self.rpm = {mid: 0 for mid in (1, 2, 3, 4)}

    @property
    def in_waiting(self):
        return len(self.buffer)

    def reset_input_buffer(self):
        self.buffer.clear()

    def _reply(self, mid, *, info):
        report = {
            "T": 20010, "typ": 115, "id": mid, "mode": self.modes[mid],
            "spd": self.rpm[mid], "tor": 0, "err": 0,
        }
        if info:
            report.update(temp=25, u8=0)
        else:
            report["pos"] = 0
        self.buffer.extend(json.dumps(report).encode() + b"\n")

    def write(self, data):
        command = json.loads(data)
        self.writes.append(command)
        kind = command["T"]
        mid = command.get("id")
        if kind == 10012:
            self.modes[mid] = command["mode"]
        elif kind == 10000:
            self.rpm[mid] = 0
            self._reply(mid, info=False)
        elif kind == 10032:
            self._reply(mid, info=True)
        elif kind == 10010:
            self.rpm[mid] = int(command["cmd"] / 2048)
            self._reply(mid, info=False)
        return len(data)

    def read(self, size):
        chunk = bytes(self.buffer[:size])
        del self.buffer[:size]
        return chunk

    def close(self):
        self.closed = True


class LiveRadio:
    def snapshot(self) -> CRSFSnapshot:
        return snapshot(arm=1811, throttle=1811, now=time.monotonic() - 0.001)


class HatIntegrationTests(unittest.TestCase):
    def test_four_current_commands_and_full_stop_use_real_driver_methods(self):
        serial_port = SimulatedHatSerial()
        cfg = settings()
        robot = Robot(cfg, LiveRadio())

        def driver_factory(_port, **kwargs):
            return DDSMHat(
                serial_port=serial_port,
                timeout=kwargs["timeout"],
                startup_delay=0,
                stop_on_close=kwargs["stop_on_close"],
            )

        with patch("robot_main.DDSMHat", side_effect=driver_factory):
            robot.open_hat()
            try:
                robot.prepare_current_mode()
                robot.drive(drive_request(robot.radio.snapshot(), cfg))
            finally:
                robot.stop_all()

        currents = [command for command in serial_port.writes if command["T"] == 10010]
        self.assertEqual([command["id"] for command in currents], [1, 2, 3, 4])
        self.assertLess(currents[0]["cmd"], 0)
        self.assertGreater(currents[1]["cmd"], 0)
        self.assertGreater(currents[2]["cmd"], 0)
        self.assertLess(currents[3]["cmd"], 0)
        zeros = [command for command in serial_port.writes if command["T"] == 10000]
        self.assertEqual([command["id"] for command in zeros[-8:]], [1, 2, 3, 4] * 2)
        heartbeat_index = next(index for index, command in enumerate(serial_port.writes) if command["T"] == 11001)
        first_four_stops = [
            index for index, command in enumerate(serial_port.writes)
            if command["T"] == 10000
        ][:4]
        self.assertLess(max(first_four_stops), heartbeat_index)
        self.assertTrue(serial_port.closed)


if __name__ == "__main__":
    unittest.main()
