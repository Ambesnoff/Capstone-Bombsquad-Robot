"""Service recovery is bounded and shutdown takes precedence over a spawn."""
from __future__ import annotations

import importlib.util
from pathlib import Path
import signal
import threading
import unittest
from unittest.mock import Mock, patch

SPEC = importlib.util.spec_from_file_location(
    "robot_service", Path(__file__).resolve().parents[1] / "deploy/run_service.py")
service = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(service)


class RecoveryTests(unittest.TestCase):
    def test_repeated_failure_has_five_starts_and_preserves_final_exit(self):
        stop = Mock()
        stop.is_set.return_value = False
        stop.wait.return_value = False
        children = [Mock() for _ in range(5)]
        for child in children:
            child.wait.return_value = 7
        with patch.object(service.threading, "Event", return_value=stop), \
                patch.object(service.signal, "signal"), \
                patch.object(service.subprocess, "Popen", side_effect=children) as spawn:
            self.assertEqual(service.main([]), 7)
        self.assertEqual(spawn.call_count, 5)
        self.assertEqual([call.args[0] for call in stop.wait.call_args_list], [2, 4, 8, 16])

    def test_signal_during_spawn_terminates_new_child_without_restarting(self):
        handlers = {}
        child = Mock()
        child.poll.return_value = None
        child.wait.return_value = -signal.SIGTERM

        def spawn(*_args, **_kwargs):
            handlers[signal.SIGTERM]()
            return child

        real_event = threading.Event
        with patch.object(service.signal, "signal", side_effect=lambda sig, handler: handlers.update({sig: handler})), \
                patch.object(service.subprocess, "Popen", side_effect=spawn) as started, \
                patch.object(service.threading, "Event", side_effect=real_event):
            self.assertEqual(service.main([]), 0)
        started.assert_called_once()
        child.terminate.assert_called_once()


if __name__ == "__main__":
    unittest.main()
