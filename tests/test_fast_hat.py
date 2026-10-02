"""Protocol-v2 transport, replay, acknowledgment and stop-race regressions."""
from dataclasses import replace
import math
import struct
import threading
import time
import unittest
from unittest.mock import patch
from enum import EnumMeta

from fast_hat import (
    Capability, ConfigResult, FastConfig, FastHat, FastHatError, FrameParser,
    FrameType, HatState, HoldFlag, MAX_STATUS_BACKLOG, Profile, Reason,
    StopState, Validity, crc16_ccitt_false, decode_status, encode_frame,
)
from protocol_defs import (
    CONFIG_PREFIX, CONFIG_STRUCT, FaultCode, STATUS_HEADER_FIELDS, STATUS_HEADER_STRUCT,
    STATUS_LENGTH, TARGETS_STRUCT, WHEEL_FIELDS, WHEEL_OFFSETS, WHEEL_STRUCT,
)


def status_payload(ack=1, state=HatState.DISARMED, *, wheel_age=4, fault=0,
                   wheel_overrides=None, **changes):
    h = dict.fromkeys(STATUS_HEADER_FIELDS, 0)
    h.update(boot_id=0x1020304050607080, host_session=123, capabilities=255,
             build_id=0x12345678, config_id=0, accepted_seq=ack, applied_seq=ack,
             state=state, fault_code=fault, stop_state=StopState.CONFIRMED,
             sweep_us=18000, command_age_ms=8, boost_capacity_ms=20000,
             boost_remaining_ms=12500, boost_refill_remaining_ms=22500,
             hello_nonce=11)
    if state in (HatState.ARMED, HatState.SETTLING, HatState.HOLDING):
        h['stop_state'] = StopState.NONE
    h.update(changes)
    w = dict(target_centi_rpm=-1250, rpm=0, current_ma=-180, position_raw=32000,
             temp_c=25, effective_cap_ma=800, hold_cap_ma=300, age_ms=wheel_age,
             temp_age_ms=400, error=0, mode=2, validity=15, reason_flags=0)
    w.update(wheel_overrides or {})
    return (STATUS_HEADER_STRUCT.pack(*(h[n] for n in STATUS_HEADER_FIELDS)) +
            b''.join(WHEEL_STRUCT.pack(*(w[n] for n in WHEEL_FIELDS)) for _ in range(4)))


class FakeSerial:
    def __init__(self, *, auto_ack=False):
        self.timeout = .01
        self.write_timeout = .15
        self.auto_ack = auto_ack
        self.writes = []
        self._input = bytearray()
        self._condition = threading.Condition()
        self.closed = False
        self.read_error = False
        self.status_sequence = 41
        self.boot_id = 0x1020304050607080
        self.session = 122
        self.nonce = 0
        self.config_id = 0
        self.config_ack_seq = 0
        self.capabilities = 255

    @property
    def in_waiting(self):
        with self._condition:
            return len(self._input)

    def reset_input_buffer(self):
        with self._condition:
            self._input.clear()

    def inject(self, data):
        with self._condition:
            self._input.extend(data)
            self._condition.notify_all()

    def read(self, size):
        with self._condition:
            self._condition.wait_for(lambda: self._input or self.closed or self.read_error,
                                     timeout=self.timeout)
            if self.read_error:
                raise OSError('receiver unplugged')
            chunk = bytes(self._input[:size])
            del self._input[:size]
            return chunk

    def response(self, frame, *, inject=True, **changes):
        state = HatState.DISARMED
        if frame.kind == FrameType.HELLO:
            self.nonce, = struct.unpack('<Q', frame.payload)
            self.session += 1
            self.config_id = 0
        if frame.kind == FrameType.CONFIG:
            self.config_id = CONFIG_PREFIX.unpack_from(frame.payload)[2]
            self.config_ack_seq = frame.sequence
        if frame.kind in (FrameType.ARM, FrameType.TARGETS):
            state = HatState.ARMED
        settings = dict(boot_id=self.boot_id, host_session=self.session,
                        hello_nonce=self.nonce, capabilities=self.capabilities,
                        config_id=self.config_id, config_ack_seq=self.config_ack_seq,
                        config_result=ConfigResult.APPLIED if self.config_id else ConfigResult.NONE)
        settings.update(changes)
        state = settings.pop('state', state)
        raw = encode_frame(FrameType.STATUS, self.status_sequence,
                           status_payload(frame.sequence, state, **settings))
        self.status_sequence = (self.status_sequence + 1) & 0xffff
        if inject:
            self.inject(raw)
        return raw

    def write(self, data):
        self.writes.append(data)
        if self.auto_ack:
            self.response(FrameParser().feed(data)[0])
        return len(data)

    def close(self):
        with self._condition:
            self.closed = True
            self._condition.notify_all()


def ready(hat, config=None):
    hat.wait_ack(hat.hello())
    hat.wait_ack(hat.send_config(config or FastConfig()))


def wait_for(predicate, timeout=.3):
    until = time.monotonic() + timeout
    while not predicate() and time.monotonic() < until:
        time.sleep(.001)
    if not predicate():
        raise AssertionError('Condition not reached')


class FrameTests(unittest.TestCase):
    def test_crc_reference_vector(self):
        self.assertEqual(crc16_ccitt_false(b'123456789'), 0x29B1)

    def test_fragmentation_corruption_v1_and_noise_resynchronize(self):
        parser = FrameParser()
        good = encode_frame(FrameType.HELLO, 314, struct.pack('<Q', 123))
        bad = bytearray(encode_frame(FrameType.STATUS_REQ, 7)); bad[-1] ^= 128
        invalid_length = b'\xa5\x5a\x02\x80\x01\x00\xff'
        self.assertEqual(parser.feed(b'noise\xa5'), [])
        self.assertEqual(parser.feed(good[1:5]), [])
        self.assertEqual(len(parser.feed(good[5:])), 1)
        v1 = bytearray(good); v1[2] = 1
        self.assertEqual(len(parser.feed(bytes(bad) + invalid_length + bytes(v1) + good)), 1)
        self.assertEqual((parser.bad_crc, parser.bad_length, parser.bad_version), (1, 1, 1))
        self.assertLessEqual(len(parser.buffer), 249)

    def test_status_validity_temperature_age_and_stop_are_independent(self):
        raw = status_payload(22, HatState.FAULT, fault=10, stop_state=StopState.CONFIRMED,
                             wheel_overrides=dict(validity=Validity.RPM | Validity.CURRENT,
                                                  temp_age_ms=65535))
        s = decode_status(raw, received_at=10)
        self.assertEqual(len(raw), STATUS_LENGTH)
        self.assertEqual(s.ack_seq, 22)
        self.assertEqual(s.wheels[0].target_rpm, -12.5)
        self.assertIsNone(s.wheels[0].temp_c)
        self.assertIsNone(s.wheels[0].position_raw)
        self.assertIsNone(s.wheels[0].temp_age_ms)
        self.assertTrue(s.stationary())
        self.assertTrue(s.stop_confirmed)
        self.assertFalse(s.temperature_fresh(750))
        self.assertFalse(s.motion_enabled)

    def test_valid_speed_required_for_stationary_even_when_value_zero(self):
        s = decode_status(status_payload(wheel_overrides=dict(validity=Validity.TEMPERATURE)))
        self.assertFalse(s.stationary())
        self.assertTrue(s.temperature_fresh(750))

    def test_fault_decoding_with_pre_312_enum_containment(self):
        def legacy_contains(enum, member):
            if not isinstance(member, enum):
                raise TypeError("Raw values are unsupported before Python 3.12")
            return member.name in enum.__members__

        with patch.object(EnumMeta, '__contains__', legacy_contains):
            for fault in FaultCode:
                with self.subTest(fault=fault):
                    status = decode_status(status_payload(fault=int(fault)))
                    self.assertIs(status.fault_code, fault)
            with self.assertRaises(ValueError):
                decode_status(status_payload(fault=255))

    def test_status_rejects_invalid_fields(self):
        for changes in (dict(state=99), dict(fault=255), dict(stop_state=99),
                        dict(requested_profile=3), dict(boost_remaining_ms=21000),
                        dict(fault_wheel=5), dict(wheel_overrides=dict(effective_cap_ma=2701)),
                        dict(wheel_overrides=dict(validity=16))):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                decode_status(status_payload(**changes))

    def test_implausible_temperature_hides_only_that_wheel(self):
        # One faulty or cold-encoded sensor byte must not hide HAT fault/reason flags.
        for value in (251, -41):
            with self.subTest(temp_c=value):
                raw = bytearray(status_payload(9, HatState.FAULT, fault=FaultCode.OVERTEMPERATURE,
                                               fault_wheel=2, reason_flags=Reason.THERMAL_STOP))
                struct.pack_into('<h', raw, STATUS_HEADER_STRUCT.size + WHEEL_STRUCT.size + WHEEL_OFFSETS['temp_c'], value)
                s = decode_status(bytes(raw))
                self.assertEqual((s.fault_code, s.fault_wheel, s.reason_flags), (FaultCode.OVERTEMPERATURE, 2, Reason.THERMAL_STOP))
                self.assertIsNone(s.wheels[1].temp_c)
                self.assertEqual(s.wheels[1].validity, Validity.RPM | Validity.CURRENT | Validity.POSITION)
                self.assertEqual([w.temp_c for w in s.wheels], [25, None, 25, 25])
                self.assertTrue(s.feedback_fresh(150)); self.assertFalse(s.temperature_fresh(1500))


class ConfigTests(unittest.TestCase):
    def test_candidate_caps_and_independent_firmware_ceiling(self):
        c = FastConfig()
        self.assertEqual((c.gentle_current_ma, c.normal_current_ma, c.boost_current_ma), (800,1500,2500))
        self.assertEqual(c.max_current_ma,2700)
        self.assertEqual(len(c.payload()),CONFIG_STRUCT.size)
        self.assertNotEqual(c.configuration_id,replace(c,max_current_ma=2600).configuration_id)
        self.assertEqual((c.boost_capacity_ms,c.boost_refill_ms,c.temp_warn_c,c.temp_derate_c,c.temp_limit_c),
                         (20000,60000,50,55,65))

    def test_strict_types_ranges_and_cross_constraints(self):
        for field,value in [('max_rpm',True),('max_current_ma',2701),('temp_poll_ms',501),
                            ('boost_refill_ms',59999),('hold_enabled',1),('max_rpm','40'),
                            ('temp_warn_c',55),('temp_release_c',55),('temp_boost_stale_ms',500),
                            ('hold_temp_c',40),('feedback_timeout_ms',300),('disarmed_hold_enabled',True)]:
            c = replace(FastConfig(), **{field:value})
            if field == 'disarmed_hold_enabled':
                c = replace(c,hold_enabled=False)
            with self.subTest(field=field,value=value), self.assertRaises(ValueError):
                c.payload()


class FastHatTests(unittest.TestCase):
    def test_no_motion_on_open_and_v2_session_configuration_target_layout(self):
        fake = FakeSerial(auto_ack=True)
        with FastHat('test',timeout=.2,serial_port=fake) as hat:
            self.assertEqual(fake.writes,[])
            ready(hat)
            cframe = FrameParser().feed(fake.writes[-1])[0]
            self.assertEqual(cframe.payload[20:],FastConfig().payload())
            self.assertEqual(CONFIG_PREFIX.unpack_from(cframe.payload),
                             (fake.boot_id,fake.session,FastConfig().configuration_id))
            with self.assertRaises(FastHatError):
                hat.targets((-12.25,12.25,12.25,-12.25),Profile.NORMAL)
            hat.wait_ack(hat.arm())
            hat.wait_ack(hat.targets((-12.25,12.25,12.25,-12.25),Profile.NORMAL))
            target = FrameParser().feed(fake.writes[-1])[0]
            self.assertEqual(TARGETS_STRUCT.unpack(target.payload),
                             (fake.boot_id,fake.session,-1225,1225,1225,-1225,1))
            self.assertIsNotNone(hat.stats().last_rtt_ms)
            self.assertTrue(hat.wait_ack(hat.stop()).stop_confirmed)
            with self.assertRaises(FastHatError):
                hat.targets((0,0,0,0),Profile.GENTLE)
        self.assertTrue(fake.closed)
        self.assertEqual(FrameParser().feed(fake.writes[-1])[0].kind,FrameType.STOP)

    def test_preconditions_and_invalid_config_do_not_write(self):
        fake = FakeSerial()
        with FastHat('test',serial_port=fake) as hat:
            with self.assertRaises(ValueError):hat.send_config(replace(FastConfig(),max_rpm=201))
            with self.assertRaises(FastHatError):hat.arm()
            with self.assertRaises(FastHatError):hat.send_config(FastConfig())
            self.assertEqual(fake.writes,[])

    def test_bad_status_and_wrong_hello_nonce_cannot_establish_session(self):
        fake = FakeSerial()
        with FastHat('test',timeout=.05,serial_port=fake) as hat:
            seq = hat.hello(); frame = FrameParser().feed(fake.writes[-1])[0]
            fake.inject(encode_frame(FrameType.STATUS,1,b'bad'))
            fake.response(frame,hello_nonce=123)
            with self.assertRaisesRegex(FastHatError,'No HAT ACK'):hat.wait_ack(seq)
            self.assertEqual(hat.stats().bad_status,1)
            self.assertIsNone(hat.snapshot())

    def test_missing_capability_refuses_hello(self):
        fake = FakeSerial(auto_ack=True);fake.capabilities=127
        with FastHat('test',timeout=.2,serial_port=fake) as hat:
            with self.assertRaisesRegex(FastHatError,'capabilities'):hat.wait_ack(hat.hello())
            with self.assertRaises(FastHatError):hat.arm()

    def test_config_ack_exact_identity_required(self):
        fake = FakeSerial(auto_ack=True)
        with FastHat('test',timeout=.2,serial_port=fake) as hat:
            hat.wait_ack(hat.hello());fake.auto_ack=False
            seq = hat.send_config(FastConfig());frame=FrameParser().feed(fake.writes[-1])[0]
            fake.response(frame,config_id=12345)
            with self.assertRaisesRegex(FastHatError,'exact stopped CONFIG'):hat.wait_ack(seq)
            with self.assertRaises(FastHatError):hat.arm()

    def test_config_rejection_is_explicit_without_advancing_motion_sequence(self):
        fake=FakeSerial(auto_ack=True)
        with FastHat('test',timeout=.2,serial_port=fake) as hat:
            hello=hat.hello();hat.wait_ack(hello);fake.auto_ack=False
            seq=hat.send_config(FastConfig());frame=FrameParser().feed(fake.writes[-1])[0]
            fake.response(frame,accepted_seq=hello,applied_seq=hello,
                          config_result=ConfigResult.REJECTED_VALUE,config_id=0)
            with self.assertRaisesRegex(FastHatError,'rejected CONFIG.*REJECTED_VALUE'):
                hat.wait_ack(seq)
            self.assertEqual(hat.snapshot().accepted_seq,hello)
            with self.assertRaises(FastHatError):hat.arm()

    def test_arm_ack_requires_enabled_state(self):
        fake=FakeSerial(auto_ack=True)
        with FastHat('test',timeout=.2,serial_port=fake) as hat:
            ready(hat);fake.auto_ack=False
            seq=hat.arm();fake.response(FrameParser().feed(fake.writes[-1])[0],state=HatState.DISARMED)
            with self.assertRaisesRegex(FastHatError,'without entering ARMED'):hat.wait_ack(seq)

    def test_stop_invalidates_pending_arm_ack_before_or_after_arm_write(self):
        fake=FakeSerial(auto_ack=True)
        with FastHat('test',timeout=.2,serial_port=fake) as hat:
            ready(hat);fake.auto_ack=False
            arm=hat.arm();aframe=FrameParser().feed(fake.writes[-1])[0]
            stop=hat.stop()
            fake.response(aframe)
            with self.assertRaisesRegex(FastHatError,'STOP superseded'):hat.wait_ack(arm)
            with self.assertRaises(FastHatError):hat.targets((1,1,1,1),Profile.GENTLE)
            with self.assertRaises(FastHatError):hat.arm()
            self.assertEqual(FrameParser().feed(fake.writes[-1])[0].sequence,stop)

    def test_stop_precedes_arm_queued_for_write_lock(self):
        fake=FakeSerial(auto_ack=True)
        with FastHat('test',timeout=.2,serial_port=fake) as hat:
            ready(hat);hat._write_lock.acquire();errors=[]
            thread=threading.Thread(target=lambda:self.capture_arm(hat,errors));thread.start()
            time.sleep(.01)
            stopper=threading.Thread(target=hat.stop);stopper.start()
            wait_for(lambda:hat._stop_pending)
            hat._write_lock.release();thread.join(.3);stopper.join(.3)
            self.assertEqual(len(errors),1)
            self.assertIn('STOP superseded',str(errors[0]))
            self.assertNotIn(FrameType.ARM,[FrameParser().feed(w)[0].kind for w in fake.writes])

    @staticmethod
    def capture_arm(hat,errors):
        try:hat.arm()
        except FastHatError as exc:errors.append(exc)

    def test_fault_does_not_erase_stop_confirmation_or_block_stop_ack(self):
        fake=FakeSerial(auto_ack=True)
        with FastHat('test',timeout=.2,serial_port=fake) as hat:
            ready(hat);fake.auto_ack=False;seq=hat.stop()
            fake.response(FrameParser().feed(fake.writes[-1])[0],state=HatState.FAULT,fault=6)
            status=hat.wait_ack(seq)
            self.assertEqual(status.fault_code,6)
            self.assertTrue(status.stop_confirmed)

    def test_duplicate_old_status_and_rollover(self):
        fake=FakeSerial(auto_ack=True);fake.status_sequence=65535
        with FastHat('test',timeout=.2,serial_port=fake) as hat:
            hat.wait_ack(hat.hello());ready_config=hat.send_config(FastConfig());hat.wait_ack(ready_config)
            current=hat.snapshot();frame=FrameParser().feed(fake.writes[-1])[0]
            fake.status_sequence=0;fake.response(frame);fake.status_sequence=65535;fake.response(frame)
            wait_for(lambda:hat.stats().old_status==2)
            self.assertEqual(hat.snapshot().received_at,current.received_at)

    def test_boot_identity_change_latches_motion_inhibition_then_new_hello_recovers(self):
        fake=FakeSerial(auto_ack=True)
        with FastHat('test',timeout=.2,serial_port=fake) as hat:
            ready(hat);hat.wait_ack(hat.arm())
            fake.boot_id += 1;fake.status_sequence=0
            fake.response(FrameParser().feed(fake.writes[-1])[0])
            wait_for(lambda:hat._identity_error is not None)
            with self.assertRaises(FastHatError):hat.targets((1,1,1,1),Profile.GENTLE)
            ready(hat)
            self.assertEqual(hat.snapshot().boot_id,fake.boot_id)
            self.assertFalse(hat.snapshot().motion_enabled)

    def test_corrupt_status_ignored_fresh_status_acknowledged(self):
        fake=FakeSerial()
        with FastHat('test',timeout=.2,serial_port=fake) as hat:
            seq=hat.hello();frame=FrameParser().feed(fake.writes[-1])[0]
            bad=bytearray(fake.response(frame,inject=False));bad[-1]^=1
            fake.inject(bytes(bad));fake.response(frame)
            self.assertEqual(hat.wait_ack(seq).ack_seq,seq)
            self.assertEqual(hat.stats().bad_crc,1)

    def test_reader_error_still_allows_best_effort_stop(self):
        fake=FakeSerial()
        with FastHat('test',timeout=.1,serial_port=fake) as hat:
            fake.read_error=True;fake.inject(b'x')
            wait_for(lambda:hat.stats().reader_error is not None)
            with self.assertRaises(FastHatError):hat.hello()
            seq=hat.stop()
            self.assertEqual(FrameParser().feed(fake.writes[-1])[0].sequence,seq)

    def test_receive_backlog_is_discarded_and_reader_keeps_running(self):
        fake=FakeSerial(auto_ack=True)
        with FastHat('test',timeout=.2,serial_port=fake) as hat:
            ready(hat);hat.wait_ack(hat.arm());live=hat.snapshot()
            # Reports queued while the reader was delayed are old; none may become live.
            arm=FrameParser().feed(fake.writes[-1])[0]
            backlog=b''.join(fake.response(arm,inject=False) for _ in range(5))
            self.assertGreater(len(backlog),MAX_STATUS_BACKLOG)
            fake.inject(backlog)
            wait_for(lambda:hat.stats().backlog_discards==1)
            self.assertIs(hat.snapshot(),live)
            self.assertIsNone(hat.stats().reader_error);self.assertTrue(hat._reader.is_alive())
            with self.assertRaisesRegex(FastHatError,'ARM must be acknowledged'):
                hat.targets((1,1,1,1),Profile.GENTLE)
            # Fresh reports still arrive; a new ARM is required before TARGETS.
            self.assertGreater(hat.wait_ack(hat.request_status()).received_at,live.received_at)
            hat.wait_ack(hat.arm());hat.wait_ack(hat.targets((1,1,1,1),Profile.GENTLE))

    def test_nonfinite_targets_and_invalid_profile_rejected(self):
        fake=FakeSerial(auto_ack=True)
        with FastHat('test',timeout=.2,serial_port=fake) as hat:
            ready(hat);hat.wait_ack(hat.arm())
            for value in (math.nan,math.inf,True,'1',41):
                with self.subTest(value=value),self.assertRaises(ValueError):
                    hat.targets((value,0,0,0),Profile.NORMAL)
            for profile in (True,300,'Boost'):
                with self.subTest(profile=profile),self.assertRaises(ValueError):
                    hat.targets((0,0,0,0),profile)


if __name__=='__main__':unittest.main()
