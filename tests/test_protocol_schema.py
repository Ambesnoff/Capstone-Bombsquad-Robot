"""Checked definitions, fixed golden frames and cross-language config rejection."""
import json
from pathlib import Path
import re
import struct
import subprocess
import sys
import unittest

from fast_hat import FastConfig, FrameParser, FrameType, Profile, decode_status, encode_frame
from protocol_defs import CONFIG_FIELDS, CONFIG_PREFIX, CONFIG_SPEC, CONFIG_STRUCT, Reason, TARGETS_STRUCT, validate_config
try:from native_toolchain import build_native
except ImportError:from tests.native_toolchain import build_native

ROOT = Path(__file__).resolve().parents[1]
WIRE_MAX={'B':255,'H':65535,'I':2**32-1}


class GeneratedDefinitionTests(unittest.TestCase):
    """Pure Python: runs with or without a native compiler."""
    def test_generated_files_match_authoritative_schema(self):
        result=subprocess.run([sys.executable,str(ROOT/'generate_protocol.py'),'--check'],
                              capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)

    def test_cpp_range_checks_omit_only_bounds_implied_by_the_wire_type(self):
        # GNU GCC -Wextra rejects always-false comparisons (-Wtype-limits) under -Werror.
        header=(ROOT/'hat_firmware/robot_hat/protocol_v2.h').read_text()
        ranges='\n'.join(line for line in header.splitlines() if line.startswith('  if (c.'))
        for field in CONFIG_SPEC:
            name=field['name']
            self.assertEqual(bool(re.search(rf'c\.{name} < {field["min"]}\b',ranges)),field['min']>0,name+' min')
            self.assertEqual(bool(re.search(rf'c\.{name} > {field["max"]}\b',ranges)),
                             field['max']<WIRE_MAX[field['type']],name+' max')

    def test_reply_retry_reason_is_wire_bit_18(self):
        # Informational reason: the firmware reports a recently retried motor reply for 1000 ms.
        self.assertEqual(Reason.REPLY_RETRY,262144)

    def test_python_validator_keeps_every_bound(self):
        baseline={field['name']:field['default'] for field in CONFIG_SPEC}
        for field in CONFIG_SPEC:
            for value in (field['min']-1,field['max']+1):
                with self.subTest(field=field['name'],value=value),self.assertRaises(ValueError):
                    validate_config({**baseline,field['name']:value})


class ProtocolSchemaTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.binary=build_native(cls,ROOT/'tests/protocol_probe.cpp','protocol-probe')

    def test_fixed_golden_frames_match_independent_cpp_structs_crc_and_python(self):
        vectors=json.loads((ROOT/'tests/fixtures/protocol_v2_golden.json').read_text())
        native=subprocess.check_output([str(self.binary),'golden'],text=True).splitlines()
        self.assertEqual(native,list(vectors.values()))
        config=FastConfig()
        targets=TARGETS_STRUCT.pack(0x1020304050607080,123,-1225,1225,1225,-1225,Profile.BOOST)
        python=[encode_frame(FrameType.HELLO,0x1234,struct.pack('<Q',0x0102030405060708)),
                encode_frame(FrameType.STOP,65535),encode_frame(FrameType.TARGETS,65535,targets),
                encode_frame(FrameType.CONFIG,2,CONFIG_PREFIX.pack(0x1020304050607080,123,config.configuration_id)+config.payload())]
        self.assertEqual([value.hex() for value in python],list(vectors.values())[:4])
        status=FrameParser().feed(bytes.fromhex(vectors['status']))[0]
        self.assertEqual(status.kind,FrameType.STATUS)
        decoded=decode_status(status.payload)
        self.assertEqual((decoded.boot_id,decoded.host_session,decoded.ack_seq,decoded.applied_seq),
                         (0x1020304050607080,123,22,22))
        self.assertEqual(decoded.wheels[0].position_raw,32000)
        self.assertEqual(decoded.wheels[0].temp_age_ms,400)

    def test_python_cpp_agree_all_field_boundaries_and_relational_failures(self):
        baseline={field['name']:field['default'] for field in CONFIG_SPEC}
        cases=[baseline]
        for field in CONFIG_SPEC:
            for value in (field['min']-1,field['min'],field['max'],field['max']+1):
                # Values not representable on wire are covered by host strict-type tests.
                width=WIRE_MAX[field['type']]
                if 0<=value<=width:
                    # Wire booleans 0/1 map to typed host booleans. Host rejects
                    # integer lookalikes separately; raw 2 remains invalid here.
                    if isinstance(field['default'],bool) and value in (0,1):value=bool(value)
                    cases.append({**baseline,field['name']:value})
        # Each relationship is violated by at least one of these combinations.
        cases.extend({**baseline,**change} for change in [
            dict(gentle_current_ma=1600),dict(neutral_brake_ma=1000,max_current_ma=800),
            dict(hold_current_ma=1000,max_current_ma=800),dict(temp_release_c=50),
            dict(temp_warn_c=55),dict(temp_derate_c=65),dict(temp_release_c=49,temp_hysteresis_c=10),
            dict(temp_poll_ms=500,temp_boost_stale_ms=500),dict(temp_boost_stale_ms=750,temp_stop_stale_ms=750),
            dict(stall_target_centi_rpm=200,stall_speed_centi_rpm=200),
            dict(control_period_ms=100,feedback_timeout_ms=100),dict(feedback_timeout_ms=250,watchdog_ms=250),
            dict(disarmed_hold_enabled=True,hold_enabled=False),dict(hold_temp_c=45),dict(hold_temp_c=65,temp_limit_c=60)])
        expected=[];lines=[]
        for values in cases:
            try:validate_config(values);valid=True
            except ValueError:valid=False
            expected.append('1' if valid else '0')
            lines.append(CONFIG_STRUCT.pack(*(values[n] for n in CONFIG_FIELDS)).hex())
        result=subprocess.run([str(self.binary)],input='\n'.join(lines)+'\n',capture_output=True,text=True,check=True)
        actual=result.stdout.splitlines()
        self.assertEqual(len(actual),len(expected))
        for index,(want,got) in enumerate(zip(expected,actual)):
            with self.subTest(case=index,changed={k:v for k,v in cases[index].items() if v!=baseline[k]}):
                self.assertEqual(got,want)


if __name__=='__main__':unittest.main()
