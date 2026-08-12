#!/usr/bin/env python3

# Copyright 2026 Mentor Pi maintainers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Static checks for the bounded Mentor Pi ROS interface contract."""

from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MESSAGE_DIRECTORY = PACKAGE_ROOT / 'msg'
SERVICE_DIRECTORY = PACKAGE_ROOT / 'srv'
CONTRACT_PATH = PACKAGE_ROOT.parents[1] / 'docs' / 'framework' / (
    'ros-interface-contract.md'
)

EXPECTED_MESSAGES = {
    'BatteryState.msg',
    'BusServoCommand.msg',
    'BusServoState.msg',
    'ButtonEvent.msg',
    'BuzzerCommand.msg',
    'ControllerDiagnostics.msg',
    'Heartbeat.msg',
    'ImuState.msg',
    'LedCommand.msg',
    'MotorCommand.msg',
    'MotorState.msg',
    'OledCommand.msg',
    'PwmServoCommand.msg',
    'PwmServoState.msg',
    'Result.msg',
    'RgbCommand.msg',
}

EXPECTED_SERVICES = {
    'ConfigureBusServo.srv',
    'GetBusServoState.srv',
    'SetBatteryThreshold.srv',
    'SetMotorModel.srv',
    'SetMotorAdrc.srv',
    'SetPwmServoOffsets.srv',
    'StopBusServos.srv',
}

EXPECTED_RESULT_CODES = {
    'OK': 0,
    'INVALID_ARGUMENT': 1,
    'OUT_OF_RANGE': 2,
    'BUSY': 3,
    'TIMEOUT': 4,
    'IO_ERROR': 5,
    'UNSUPPORTED': 6,
    'PARTIAL': 7,
}

EXPECTED_FIXED_ARRAYS = {
    ('BusServoCommand.msg', 'servo_id'): 16,
    ('BusServoCommand.msg', 'position'): 16,
    ('ControllerDiagnostics.msg', 'mailbox_overwrites'): 7,
    ('ControllerDiagnostics.msg', 'motor_lease_expiries'): 4,
    ('ControllerDiagnostics.msg', 'motor_command_rejections'): 4,
    ('ControllerDiagnostics.msg', 'peripheral_errors'): 8,
    ('ControllerDiagnostics.msg', 'peripheral_timeouts'): 8,
    ('ControllerDiagnostics.msg', 'usart1_errors'): 4,
    ('ControllerDiagnostics.msg', 'task_missed_releases'): 6,
    ('ControllerDiagnostics.msg', 'task_max_execution_us'): 6,
    ('ControllerDiagnostics.msg', 'task_stack_high_water_bytes'): 6,
    ('ControllerDiagnostics.msg', 'task_heartbeat_age_ms'): 6,
    ('ControllerDiagnostics.msg', 'free_ram_bytes'): 2,
    ('ControllerDiagnostics.msg', 'minimum_free_ram_bytes'): 2,
    ('ImuState.msg', 'angular_velocity_rad_s'): 3,
    ('ImuState.msg', 'linear_acceleration_m_s2'): 3,
    ('MotorCommand.msg', 'target_rps'): 4,
    ('MotorState.msg', 'target_rps'): 4,
    ('MotorState.msg', 'measured_rps'): 4,
    ('MotorState.msg', 'encoder_count'): 4,
    ('PwmServoCommand.msg', 'pulse_width_us'): 4,
    ('PwmServoState.msg', 'target_pulse_width_us'): 4,
    ('PwmServoState.msg', 'output_pulse_width_us'): 4,
    ('PwmServoState.msg', 'offset_us'): 4,
    ('RgbCommand.msg', 'red'): 2,
    ('RgbCommand.msg', 'green'): 2,
    ('RgbCommand.msg', 'blue'): 2,
    ('SetMotorAdrc.srv', 'input_gain_rps_per_second_per_permille'): 4,
    ('SetMotorAdrc.srv', 'controller_bandwidth_rad_s'): 4,
    ('SetMotorAdrc.srv', 'observer_bandwidth_rad_s'): 4,
    ('SetMotorAdrc.srv', 'velocity_filter_new_weight'): 4,
    ('SetPwmServoOffsets.srv', 'offset_us'): 4,
    ('StopBusServos.srv', 'servo_id'): 16,
}

FIELD_PATTERN = re.compile(
    r'^(?P<type>[A-Za-z][A-Za-z0-9_/]*(?:<=\d+|\[\d+\])?)\s+'
    r'(?P<name>[a-z][a-z0-9_]*)$'
)
CONSTANT_PATTERN = re.compile(
    r'^(?P<type>[A-Za-z][A-Za-z0-9_/]*)\s+'
    r'(?P<name>[A-Z][A-Z0-9_]*)=(?P<value>-?\d+)$'
)
ARRAY_PATTERN = re.compile(r'^(?P<base>[^\[]+)\[(?P<size>\d+)\]$')


def semantic_lines(text):
    """Return IDL lines that affect generated constants or fields."""
    return [
        line.strip()
        for line in text.splitlines()
        if line.strip() and not line.lstrip().startswith('#')
    ]


def definitions():
    """Yield each interface file and its semantic lines."""
    for directory in (MESSAGE_DIRECTORY, SERVICE_DIRECTORY):
        for path in sorted(directory.iterdir()):
            if path.suffix in {'.msg', '.srv'}:
                yield path, semantic_lines(path.read_text(encoding='utf-8'))


def contract_schemas(contract_text):
    """Extract normative msg/srv code blocks from the Markdown contract."""
    schemas = {}
    current_name = None
    in_schema = False
    block = []

    heading_pattern = re.compile(r'^###\s+.*`(?P<name>(?:msg|srv)/[^`]+)`')
    for line in contract_text.splitlines():
        heading = heading_pattern.match(line)
        if heading:
            current_name = heading.group('name')
            continue
        if current_name is not None and line == '```text':
            in_schema = True
            block = []
            continue
        if in_schema and line == '```':
            schemas[current_name] = semantic_lines('\n'.join(block))
            current_name = None
            in_schema = False
            block = []
            continue
        if in_schema:
            block.append(line)

    return schemas


class InterfaceContractTest(unittest.TestCase):

    def test_interface_inventory(self):
        actual_messages = {path.name for path in MESSAGE_DIRECTORY.glob('*.msg')}
        actual_services = {path.name for path in SERVICE_DIRECTORY.glob('*.srv')}
        self.assertEqual(EXPECTED_MESSAGES, actual_messages)
        self.assertEqual(EXPECTED_SERVICES, actual_services)

    def test_cmake_registers_every_interface(self):
        cmake = (PACKAGE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
        registered_messages = set(re.findall(r'"msg/([^"/]+\.msg)"', cmake))
        registered_services = set(re.findall(r'"srv/([^"/]+\.srv)"', cmake))
        self.assertEqual(EXPECTED_MESSAGES, registered_messages)
        self.assertEqual(EXPECTED_SERVICES, registered_services)

    def test_package_manifest_is_an_interface_package(self):
        root = ET.parse(PACKAGE_ROOT / 'package.xml').getroot()
        self.assertEqual('mentor_pi_interfaces', root.findtext('name'))
        groups = {element.text for element in root.findall('member_of_group')}
        self.assertIn('rosidl_interface_packages', groups)
        dependencies = {
            element.text
            for tag in ('depend', 'build_depend', 'exec_depend')
            for element in root.findall(tag)
        }
        self.assertIn('builtin_interfaces', dependencies)
        self.assertIn('rosidl_default_generators', dependencies)
        self.assertIn('rosidl_default_runtime', dependencies)

    def test_schema_syntax_and_bounded_storage(self):
        actual_arrays = {}
        bounded_strings = []

        for path, lines in definitions():
            separator_count = lines.count('---')
            self.assertEqual(1 if path.suffix == '.srv' else 0, separator_count)

            for line in lines:
                if line == '---' or CONSTANT_PATTERN.match(line):
                    continue
                field = FIELD_PATTERN.match(line)
                self.assertIsNotNone(field, f'Unsupported schema syntax in {path}: {line}')
                field_type = field.group('type')
                field_name = field.group('name')

                self.assertNotIn('[]', field_type, f'Unbounded array in {path}')
                self.assertNotRegex(
                    field_type, r'\[<=\d+\]', f'Variable sequence in {path}'
                )
                if field_type.startswith('string'):
                    bounded_strings.append((path.name, field_name, field_type))
                array = ARRAY_PATTERN.match(field_type)
                if array:
                    actual_arrays[(path.name, field_name)] = int(array.group('size'))

        self.assertEqual(EXPECTED_FIXED_ARRAYS, actual_arrays)
        self.assertEqual(
            [
                ('OledCommand.msg', 'line_1', 'string<=23'),
                ('OledCommand.msg', 'line_2', 'string<=23'),
            ],
            bounded_strings,
        )

    def test_result_codes_are_stable(self):
        constants = {}
        result_path = MESSAGE_DIRECTORY / 'Result.msg'
        for line in semantic_lines(result_path.read_text(encoding='utf-8')):
            match = CONSTANT_PATTERN.match(line)
            if match:
                constants[match.group('name')] = int(match.group('value'))
        self.assertEqual(EXPECTED_RESULT_CODES, constants)

    def test_custom_field_types_resolve(self):
        known_messages = {path.stem for path in MESSAGE_DIRECTORY.glob('*.msg')}
        for path, lines in definitions():
            for line in lines:
                field = FIELD_PATTERN.match(line)
                if not field:
                    continue
                field_type = field.group('type')
                if not field_type.startswith('mentor_pi_interfaces/'):
                    continue
                referenced_type = field_type.split('/', maxsplit=1)[1]
                self.assertIn(referenced_type, known_messages, f'Used by {path}')

    def test_sources_match_normative_contract_blocks(self):
        if not CONTRACT_PATH.is_file():
            self.skipTest('Framework contract is unavailable outside the source repository')

        schemas = contract_schemas(CONTRACT_PATH.read_text(encoding='utf-8'))
        expected_names = {
            *(f'msg/{name}' for name in EXPECTED_MESSAGES),
            *(f'srv/{name}' for name in EXPECTED_SERVICES),
        }
        self.assertEqual(expected_names, set(schemas))

        for path, lines in definitions():
            kind = 'msg' if path.suffix == '.msg' else 'srv'
            self.assertEqual(schemas[f'{kind}/{path.name}'], lines, path.name)

    def test_controller_diagnostics_payload_is_388_bytes(self):
        primitive_layout = {
            'bool': (1, 1),
            'int8': (1, 1),
            'uint8': (1, 1),
            'int16': (2, 2),
            'uint16': (2, 2),
            'int32': (4, 4),
            'uint32': (4, 4),
            'float32': (4, 4),
            'int64': (8, 8),
            'uint64': (8, 8),
            'float64': (8, 8),
            'builtin_interfaces/Time': (8, 4),
        }
        offset = 0
        diagnostics_path = MESSAGE_DIRECTORY / 'ControllerDiagnostics.msg'
        for line in semantic_lines(diagnostics_path.read_text(encoding='utf-8')):
            field = FIELD_PATTERN.match(line)
            if not field:
                continue
            field_type = field.group('type')
            count = 1
            array = ARRAY_PATTERN.match(field_type)
            if array:
                field_type = array.group('base')
                count = int(array.group('size'))
            size, alignment = primitive_layout[field_type]
            offset += (-offset) % alignment
            offset += size * count

        self.assertEqual(388, offset)
        self.assertEqual(392, offset + 4)  # Four-byte CDR encapsulation header.


if __name__ == '__main__':
    unittest.main()
