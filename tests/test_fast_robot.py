"""Supervisor regressions with deliberate radio transitions and protocol v2 identity."""
import csv
import json
import tempfile
import threading
import time
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from crsf import CRSFSnapshot
from fast_hat import FastHatError, FastStatus, FastWheel, REQUIRED_CAPABILITIES
from fast_robot import AsyncCsvTelemetry, FastRobot, fast_config
from protocol_defs import ConfigResult, FaultCode, HatState, Profile, StopState
from robot_main import Settings

CONFIG = Path(__file__).resolve().parents[1] / "config.example.json"


def settings():
    data = json.loads(CONFIG.read_text());data.update(motor_port="/dev/test-hat",radio_port="/dev/test-radio")
    return Settings.from_dict(data)


def radio_frame(*, arm=172, throttle=172, stop=172, lq=80, profile=172, steering=992):
    values = [992] * 16
    values[0] = steering
    values[2] = throttle;values[4] = arm;values[5] = profile;values[6] = stop;values[7] = 172
    now = time.monotonic()
    return CRSFSnapshot(tuple(values), now, lq, now, None)


class PhaseRadio:
    def __init__(self, phases): self.phases=phases;self.index=0;self.history=[]
    def snapshot(self): self.history.append(self.index);return radio_frame(**self.phases[self.index])


class FakeFastHat:
    def __init__(self):
        self.commands=[];self.sequence=0;self.state=HatState.DISARMED;self.fault=0;self.stop_state=StopState.CONFIRMED
        self.ages=(0,0,0,0);self.temps=(25,25,25,25);self.closed=False;self.config=None
        self.profile=Profile.GENTLE;self.targets_rpm=(0,0,0,0);self.boot=123;self.session=456
        self.capabilities=REQUIRED_CAPABILITIES;self.on_wait=None;self.queries=0
    def __enter__(self): return self
    def __exit__(self,*_args): self.closed=True
    def _send(self,name,*values): self.sequence+=1;self.commands.append((name,*values));return self.sequence
    def hello(self):
        self.state=HatState.DISARMED;self.stop_state=StopState.CONFIRMED;self.config=None;self.session+=1
        return self._send("hello")
    def send_config(self,config): self.config=config;return self._send("config")
    def arm(self): self.state=HatState.ARMED;self.stop_state=StopState.NONE;return self._send("arm")
    def targets(self,rpm,profile): self.targets_rpm=tuple(rpm);self.profile=profile;return self._send("targets",tuple(rpm),profile)
    def stop(self):
        self.state=HatState.FAULT if self.fault else HatState.DISARMED
        self.stop_state=StopState.CONFIRMED;self.ages=(0,0,0,0);self.targets_rpm=(0,0,0,0)
        return self._send("stop")
    def request_status(self): self.queries+=1;return self._send("status")
    def wait_ack(self,seq,timeout=None):
        if self.on_wait: self.on_wait(self.commands[-1][0])
        return self.snapshot()
    def snapshot(self):
        wheels=tuple(FastWheel(0,0,self.temps[i],0,self.ages[i],target_rpm=self.targets_rpm[i],position_raw=0,
                              effective_cap_ma=800 if self.state==HatState.ARMED else 0,temp_age_ms=0,mode=1)
                     for i in range(4))
        return FastStatus(self.sequence,self.state,self.fault,12000,0,wheels,time.monotonic(),boot_id=self.boot,
                          host_session=self.session,capabilities=self.capabilities,build_id=1122,
                          config_id=self.config.configuration_id if self.config else 0,
                          config_result=ConfigResult.APPLIED if self.config else ConfigResult.NONE,
                          stop_state=self.stop_state,requested_profile=self.profile,applied_profile=self.profile,
                          boost_capacity_ms=20000)
    def stats(self): return SimpleNamespace(last_rtt_ms=4,max_rtt_ms=5,last_status_interval_ms=20,reader_error=None)


class FastRobotTests(unittest.TestCase):
    def run_phases(self,phases,hat=None):
        hat=hat or FakeFastHat();radio=PhaseRadio(phases)
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup)
        robot=FastRobot(settings(),radio,telemetry_path=Path(tmp.name)/"data.csv",live_status=False)
        def advance(_duration):
            if radio.index==len(phases)-1: robot.request_shutdown()
            else: radio.index+=1
        with patch("fast_robot.FastHat",return_value=hat),patch("fast_robot.time.sleep",side_effect=advance): robot.run()
        return robot,hat,radio

    def test_example_profiles_fully_available_with_independent_2700ma_ceiling(self):
        cfg=fast_config(settings())
        self.assertEqual(cfg.max_current_ma,2700)
        self.assertEqual((cfg.gentle_current_ma,cfg.normal_current_ma,cfg.boost_current_ma),(800,1500,2500))

    def test_disk_backlog_drops_rows_without_blocking_or_stopping_control(self):
        writing=threading.Event();release=threading.Event()
        class SlowWriter:
            def writerow(self,row):
                if row and row[0]!="wall_time": writing.set();release.wait(1)
        with tempfile.TemporaryDirectory() as tmp:
            log=AsyncCsvTelemetry(Path(tmp)/"data.csv",max_rows=1)
            with patch("fast_robot.csv.writer",return_value=SlowWriter()):
                log.start();log.submit([1]);self.assertTrue(writing.wait(.5));log.submit([2])
                before=time.monotonic();self.assertFalse(log.submit([3]));self.assertLess(time.monotonic()-before,.05)
                self.assertEqual(log.dropped_rows,1);release.set();log.close()

    def test_storage_failure_is_visible_and_nonfatal(self):
        class FailingWriter:
            def writerow(self,row):
                if row and row[0]!="wall_time": raise OSError("disk disconnected")
        with tempfile.TemporaryDirectory() as tmp:
            log=AsyncCsvTelemetry(Path(tmp)/"data.csv")
            with patch("fast_robot.csv.writer",return_value=FailingWriter()):
                log.start();log.submit([1]);self.assertTrue(log._done.wait(.5))
                self.assertIn("disk disconnected",log.check());self.assertFalse(log.submit([2]));log.close()

    def test_session_filenames_and_metadata_are_unique_and_complete(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"data.csv"
            a=AsyncCsvTelemetry(path,metadata={"configuration":{"test":True}});b=AsyncCsvTelemetry(path)
            self.assertNotEqual(a.path,b.path)
            a.start();a.event("armed");a.close();self.assertTrue(a._done.wait(.5))
            self.assertEqual(json.loads(a.path.with_suffix(".session.json").read_text())["configuration"],{"test":True})
            self.assertIn('"armed"',a.path.with_suffix(".events.jsonl").read_text())

    def test_four_wheel_targets_and_truthful_disarmed_telemetry(self):
        robot,hat,_=self.run_phases([{"arm":172},{"arm":1811},{"arm":1811,"throttle":1811},
                                   {"arm":1811,"stop":1811},{"arm":1811,"throttle":1811}])
        self.assertEqual([c[0] for c in hat.commands].count("arm"),1)
        self.assertIn(("targets",(-40,40,40,-40),Profile.GENTLE),hat.commands)
        self.assertTrue(hat.closed)
        self.assertTrue(robot._telemetry._done.wait(.5))
        with robot.telemetry_path.open() as file: rows=list(csv.DictReader(file))
        self.assertEqual(rows[-1]["armed"],"0")
        self.assertEqual(rows[-1]["requested_rpm_1"],"0.0")
        self.assertEqual(len(rows[-1]),len(rows[0]))
        self.assertIn("temperature_age_ms_4",rows[-1])

    def test_radio_link_loss_requires_new_low_high_neutral_cycle(self):
        _,hat,_=self.run_phases([{"arm":172},{"arm":1811},{"arm":1811,"throttle":1811},
                                {"arm":1811,"lq":0},{"arm":1811,"throttle":1811},{"arm":172},
                                {"arm":1811},{"arm":1811,"throttle":1811}])
        self.assertEqual([c[0] for c in hat.commands].count("arm"),2)

    def test_original_second_snapshot_stop_race_is_handled_in_same_pass(self):
        class RacyRadio:
            calls=0
            def snapshot(self):
                self.calls+=1
                if self.calls==1: return radio_frame(arm=172)
                return radio_frame(arm=1811,stop=1811 if self.calls>=6 else 172)
        hat=FakeFastHat();radio=RacyRadio()
        with tempfile.TemporaryDirectory() as tmp:
            robot=FastRobot(settings(),radio,telemetry_path=Path(tmp)/"data.csv",live_status=False)
            steps=0
            def advance(_):
                nonlocal steps;steps+=1
                if steps>=8: robot.request_shutdown()
            with patch("fast_robot.FastHat",return_value=hat),patch("fast_robot.time.sleep",side_effect=advance): robot.run()
        kinds=[c[0] for c in hat.commands]
        self.assertIn("arm",kinds);self.assertIn("stop",kinds)
        self.assertNotIn("targets",kinds)

    def test_shutdown_during_pending_arm_ack_cannot_restore_targets(self):
        hat=FakeFastHat();radio=PhaseRadio([{"arm":172},{"arm":1811}])
        with tempfile.TemporaryDirectory() as tmp:
            robot=FastRobot(settings(),radio,telemetry_path=Path(tmp)/"data.csv",live_status=False)
            hat.on_wait=lambda kind: robot.request_shutdown() if kind=="arm" else None
            with patch("fast_robot.FastHat",return_value=hat),patch("fast_robot.time.sleep",side_effect=lambda _:setattr(radio,"index",1)): robot.run()
        kinds=[c[0] for c in hat.commands]
        self.assertEqual(kinds.count("arm"),1);self.assertNotIn("targets",kinds);self.assertEqual(kinds[-1],"stop")

    def test_fault_does_not_interrupt_stop_verification_queries(self):
        hat=FakeFastHat();robot=FastRobot(settings(),PhaseRadio([{}]),telemetry_path=Path("unused.csv"),live_status=False)
        robot._hat=hat;hat.fault=FaultCode.MOTOR_FAULT;hat.stop_state=StopState.IN_PROGRESS
        def query():
            hat.queries+=1
            if hat.queries>=3: hat.stop_state=StopState.CONFIRMED
            return 1
        hat.request_status=query
        status=robot._wait_disarmed(.5)
        self.assertEqual(hat.queries,3);self.assertEqual(status.fault_code,FaultCode.MOTOR_FAULT)

    def test_capability_and_configuration_mismatches_inhibit_motion(self):
        hat=FakeFastHat();robot=FastRobot(settings(),PhaseRadio([{}]),telemetry_path=Path("unused.csv"),live_status=False)
        robot._hat=hat;hat.config=robot.config
        hat.capabilities=0
        with self.assertRaisesRegex(RuntimeError,"capabilities"): robot._status()
        hat.capabilities=REQUIRED_CAPABILITIES;hat.config=replace(robot.config,max_rpm=41)
        with self.assertRaisesRegex(RuntimeError,"configuration"): robot._status()

    def test_inspection_fault_does_not_clear_on_fresh_link(self):
        robot=FastRobot(settings(),PhaseRadio([{}]),telemetry_path=Path("unused.csv"),live_status=False)
        hat=FakeFastHat();hat.fault=FaultCode.ABNORMAL_CURRENT
        self.assertIsNotNone(robot._inhibit_reason(hat.snapshot()));hat.fault=0
        self.assertIsNotNone(robot._inhibit_reason(hat.snapshot()))

    def test_thermal_recovery_clears_only_to_readiness_and_new_arm(self):
        robot=FastRobot(settings(),PhaseRadio([{}]),telemetry_path=Path("unused.csv"),live_status=False)
        hat=FakeFastHat();hat.fault=FaultCode.OVERTEMPERATURE
        self.assertIsNotNone(robot._inhibit_reason(hat.snapshot()));robot._clear_requests();hat.fault=0
        self.assertIsNone(robot._inhibit_reason(hat.snapshot()))
        self.assertNotEqual(robot.gate.observe(radio_frame(arm=1811),time.monotonic()),"newly_armed")


if __name__=="__main__": unittest.main()


class RadioStatusIntegrationTests(unittest.TestCase):
    def test_each_wheel_has_distinct_standard_rpm_and_temperature_sensor(self):
        from crsf import CRSFParser
        class Radio:
            frames=None
            def publish_telemetry(self,frames): self.frames=frames
        radio=Radio();robot=FastRobot(settings(),radio,telemetry_path=Path("unused.csv"),live_status=False)
        hat=FakeFastHat();robot._hat=hat
        with tempfile.TemporaryDirectory() as tmp:
            robot._telemetry=AsyncCsvTelemetry(Path(tmp)/"unused.csv")
            robot._record(hat.snapshot(),radio_frame())
        decoded=[CRSFParser().feed(f)[0] for f in radio.frames]
        self.assertEqual([f.payload[0] for f in decoded if f.frame_type==0x0C],[0,1,2,3])
        self.assertEqual([f.payload[0] for f in decoded if f.frame_type==0x0D],[4,5,6,7])
        self.assertEqual(len(decoded),9)
        text=next(f.payload for f in decoded if f.frame_type==0x21)
        self.assertIn(b"DISARMED",text);self.assertIn(b"G>G",text);self.assertIn(b"B0s",text)
        self.assertIn(b"CONFIRMED",text);self.assertTrue(text.endswith(b"\x00"))


class ArmingNeutralRaceTests(unittest.TestCase):
    def test_controls_changing_during_arm_ack_immediately_stop(self):
        for changed in ({"steering":1811},{"throttle":1811}):
            with self.subTest(changed=changed),tempfile.TemporaryDirectory() as tmp:
                hat=FakeFastHat();radio=PhaseRadio([{"arm":172},{"arm":1811}])
                robot=FastRobot(settings(),radio,telemetry_path=Path(tmp)/"data.csv",live_status=False)
                def on_ack(kind):
                    if kind=="arm": radio.phases[1].update(changed)
                hat.on_wait=on_ack
                steps=0
                def advance(_):
                    nonlocal steps;steps+=1;radio.index=1
                    if steps>4: robot.request_shutdown()
                with patch("fast_robot.FastHat",return_value=hat),patch("fast_robot.time.sleep",side_effect=advance): robot.run()
                kinds=[cmd[0] for cmd in hat.commands]
                self.assertEqual(kinds.count("arm"),1)
                self.assertNotIn("targets",kinds)
                self.assertGreaterEqual(kinds.count("stop"),2)


class StopBoundaryTests(unittest.TestCase):
    def robot(self,hat):
        robot=FastRobot(settings(),PhaseRadio([{}]),telemetry_path=Path("unused.csv"),live_status=False)
        robot._hat=hat
        return robot

    def test_cached_pre_stop_confirmed_status_cannot_confirm_new_request(self):
        hat=FakeFastHat();robot=self.robot(hat)
        cached=hat.snapshot();boundary=time.monotonic()
        seq=hat.stop();hat.snapshot=lambda: cached
        with self.assertRaisesRegex(RuntimeError,"Stop unconfirmed"):
            robot._wait_disarmed(.06,requested_at=boundary,stop_sequence=seq)
        self.assertGreaterEqual(hat.queries,1)

    def test_wrong_ack_and_pre_stop_motor_measurements_cannot_confirm(self):
        for wrong_ack,old_measurement in ((True,False),(False,True)):
            with self.subTest(wrong_ack=wrong_ack,old_measurement=old_measurement):
                hat=FakeFastHat();robot=self.robot(hat);boundary=time.monotonic();seq=hat.stop()
                fresh=hat.snapshot()
                if wrong_ack: fresh=replace(fresh,ack_seq=seq-1)
                if old_measurement: fresh=replace(fresh,wheels=tuple(replace(w,age_ms=100) for w in fresh.wheels))
                hat.snapshot=lambda: fresh
                with self.assertRaisesRegex(RuntimeError,"Stop unconfirmed"):
                    robot._wait_disarmed(.06,requested_at=boundary,stop_sequence=seq)

    def test_fresh_matching_stop_ack_with_postrequest_feedback_confirms_even_faulted(self):
        hat=FakeFastHat();robot=self.robot(hat);boundary=time.monotonic();seq=hat.stop();hat.fault=FaultCode.MOTOR_FAULT
        status=robot._wait_disarmed(.1,requested_at=boundary,stop_sequence=seq)
        self.assertEqual(status.accepted_seq,seq);self.assertEqual(status.fault_code,FaultCode.MOTOR_FAULT)

    def test_bad_motor_frame_recovers_to_readiness_with_fresh_arm_still_required(self):
        hat=FakeFastHat();robot=self.robot(hat);hat.fault=FaultCode.BAD_MOTOR_FRAME
        self.assertIsNotNone(robot._inhibit_reason(hat.snapshot()));robot._clear_requests();hat.fault=0
        self.assertIsNone(robot._inhibit_reason(hat.snapshot()))
        self.assertNotEqual(robot.gate.observe(radio_frame(arm=1811),time.monotonic()),"newly_armed")

    def test_pending_fault_verification_queries_do_not_repeat_stop_generations(self):
        hat=FakeFastHat();phases=[{"arm":172},{"arm":1811},{"arm":1811,"throttle":1811}]+[{"arm":1811}]*8
        radio=PhaseRadio(phases)
        def fault_after_target(rpm,profile):
            seq=hat._send("targets",tuple(rpm),profile)
            hat.fault=FaultCode.MOTOR_TIMEOUT;hat.state=HatState.FAULT;hat.stop_state=StopState.IN_PROGRESS
            return seq
        hat.targets=fault_after_target
        with tempfile.TemporaryDirectory() as tmp:
            robot=FastRobot(settings(),radio,telemetry_path=Path(tmp)/"data.csv",live_status=False)
            def advance(_):
                if radio.index==len(phases)-1: robot.request_shutdown()
                else: radio.index+=1
            with patch("fast_robot.FastHat",return_value=hat),patch("fast_robot.time.sleep",side_effect=advance): robot.run()
        self.assertGreater(hat.queries,3)
        # Only final shutdown starts a new explicit STOP; pending HAT fault stop
        # retains its verification timer throughout all inhibited control passes.
        self.assertEqual([c[0] for c in hat.commands].count("stop"),1)


class ShutdownStatusTests(unittest.TestCase):
    def shutdown(self, *, confirmed, live_status=False, before_shutdown=None):
        observed = []
        class StoppingHat(FakeFastHat):
            def stop(self):
                observed.append(robot.snapshot())
                self.state=HatState.STOPPING;self.stop_state=StopState.IN_PROGRESS
                self.targets_rpm=(0,0,0,0)
                return self._send("stop")
            def request_status(self):
                observed.append(robot.snapshot())
                self.queries+=1
                if self.queries>=2:
                    self.stop_state=StopState.CONFIRMED if confirmed else StopState.UNCONFIRMED
                    if confirmed: self.state=HatState.DISARMED
                return self.sequence
        hat=StoppingHat();radio=PhaseRadio([{"arm":172},{"arm":1811},{"arm":1811,"throttle":1811}])
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup)
        robot=FastRobot(replace(settings(),live_status_hz=.1),radio,
                        telemetry_path=Path(tmp.name)/"data.csv",live_status=live_status)
        original_wait=robot._wait_disarmed;original_sleep=time.sleep
        def wait(timeout,**kwargs):
            return original_wait(.16 if kwargs else timeout,**kwargs)
        def advance(duration):
            if robot._shutdown.is_set():
                original_sleep(duration)
            elif radio.index==len(radio.phases)-1:
                observed.append(robot.snapshot())
                if before_shutdown: before_shutdown(robot)
                robot.request_shutdown()
            else:
                radio.index+=1
        with patch("fast_robot.FastHat",return_value=hat), \
                patch.object(robot,"_wait_disarmed",side_effect=wait), \
                patch("fast_robot.time.sleep",side_effect=advance):
            if confirmed:
                robot.run()
            else:
                with self.assertRaisesRegex(FastHatError,"Stop unconfirmed"):
                    robot.run()
        return robot,hat,observed

    def test_shutdown_publishes_requested_progress_and_both_terminal_outcomes(self):
        for confirmed in (True,False):
            with self.subTest(confirmed=confirmed):
                robot,hat,observed=self.shutdown(confirmed=confirmed)
                self.assertTrue(observed[0]["armed"])
                self.assertEqual(observed[0]["stop"],"NONE")
                # The transport sees public stop intent before writing STOP.
                self.assertFalse(observed[1]["armed"])
                self.assertEqual(observed[1]["stop"],"REQUESTED")
                self.assertEqual(observed[1]["condition"],"STOP REQUESTED")
                progress=observed[2]
                self.assertEqual(progress["hat_state"],"STOPPING")
                self.assertEqual(progress["stop"],"IN_PROGRESS")
                self.assertFalse(progress["armed"])
                terminal=robot.snapshot()
                outcome="CONFIRMED" if confirmed else "UNCONFIRMED"
                self.assertEqual(terminal["stop"],outcome)
                self.assertEqual(terminal["condition"],f"STOP {outcome}")
                self.assertFalse(terminal["armed"])
                if confirmed:
                    self.assertIsNone(terminal["inspection_fault"])
                else:
                    self.assertIn("STOP UNCONFIRMED",terminal["inspection_fault"])
                self.assertTrue(hat.closed)

    def test_display_delivers_both_terminal_outcomes_without_periodic_tick(self):
        for confirmed in (True,False):
            with self.subTest(confirmed=confirmed):
                lines=[];warnings=[];consumer_threads=[]
                def logged(format,*args):
                    consumer_threads.append(threading.get_ident())
                    lines.append(format % args)
                with patch("fast_robot.LOG.info",side_effect=logged), \
                        patch("fast_robot.LOG.warning",side_effect=lambda format,*args:warnings.append(format % args)):
                    robot,_,_=self.shutdown(confirmed=confirmed,live_status=True)
                    robot._display_thread.join(.5)
                self.assertFalse(robot._display_thread.is_alive())
                outcome="CONFIRMED" if confirmed else "UNCONFIRMED"
                self.assertTrue(any(f"stop={outcome}" in line for line in lines),lines)
                self.assertEqual(set(consumer_threads),{robot._display_thread.ident})
                if not confirmed:
                    self.assertTrue(any("STOP UNCONFIRMED" in line for line in warnings),warnings)

    def test_stalled_display_keeps_shutdown_bounded_and_drains_terminal_after_release(self):
        rendering=threading.Event();release=threading.Event();terminal=threading.Event()
        lines=[]
        def logged(format,*args):
            text=format % args
            if not rendering.is_set():
                rendering.set();release.wait(2)
            lines.append(text)
            if "stop=CONFIRMED" in text: terminal.set()
        def stall_armed_display(robot):
            robot._display_wake.set()
            self.assertTrue(rendering.wait(.5))
        try:
            with patch("fast_robot.LOG.info",side_effect=logged),patch("fast_robot.LOG.warning"):
                began=time.monotonic()
                robot,_,_=self.shutdown(confirmed=True,live_status=True,before_shutdown=stall_armed_display)
                self.assertLess(time.monotonic()-began,.8)
                self.assertEqual(robot.snapshot()["stop"],"CONFIRMED")
                self.assertTrue(robot._display_thread.is_alive())
                release.set()
                self.assertTrue(terminal.wait(.5),lines)
                robot._display_thread.join(.5)
                self.assertFalse(robot._display_thread.is_alive())
        finally:
            release.set()

    def test_stop_transport_failure_publishes_unconfirmed_inspection_fault(self):
        hat=FakeFastHat();robot=FastRobot(settings(),PhaseRadio([{}]),telemetry_path=Path("unused.csv"),live_status=False)
        robot._hat=hat;hat.config=robot.config;hat.arm()
        robot._record(hat.snapshot(),radio_frame(arm=1811))
        def failed_stop():
            self.assertEqual(robot.snapshot()["stop"],"REQUESTED")
            raise FastHatError("STOP write failed")
        hat.stop=failed_stop
        with self.assertRaisesRegex(FastHatError,"STOP write failed"):
            robot._safe_stop(wait_ack=True)
        terminal=robot.snapshot()
        self.assertEqual(terminal["stop"],"UNCONFIRMED")
        self.assertIn("STOP write failed",terminal["inspection_fault"])

    def test_repeated_stop_progress_refreshes_snapshot_without_flooding_display(self):
        hat=FakeFastHat();robot=FastRobot(settings(),PhaseRadio([{}]),telemetry_path=Path("unused.csv"),live_status=False)
        hat.state=HatState.STOPPING;hat.stop_state=StopState.IN_PROGRESS
        status=hat.snapshot()
        robot._publish_stop(StopState.IN_PROGRESS,status=status)
        self.assertTrue(robot._display_wake.is_set())
        robot._display_wake.clear()
        refreshed=replace(status,wheels=tuple(replace(w,rpm=1) for w in status.wheels))
        robot._publish_stop(StopState.IN_PROGRESS,status=refreshed)
        self.assertEqual(robot.snapshot()["wheels"][0]["rpm"],1)
        self.assertFalse(robot._display_wake.is_set())
        robot._publish_stop(StopState.CONFIRMED,status=status)
        self.assertTrue(robot._display_wake.is_set())
