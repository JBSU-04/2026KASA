#!/usr/bin/env python3
"""Exercise actual sender/receiver routing without opening network sockets."""
import importlib.util
import pathlib
import struct
import unittest
from unittest.mock import Mock, patch

import yaml
from morai_msgs.msg import CtrlCmd
from morai_udp_bridge.protocol import unpack_packet

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('competition_bridge', ROOT / 'scripts/morai_udp_bridge_node.py')
bridge = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bridge)
CONFIG = yaml.safe_load((ROOT / 'config/morai_25s4.yaml').read_text())


def parameter(name, default=None):
    value = CONFIG
    for part in name.lstrip('~').split('/'):
        if not isinstance(value, dict) or part not in value:
            return default
        value = value[part]
    return value


class CompetitionTransportTest(unittest.TestCase):
    def test_control_uses_team_source_port_and_simulator_destination(self):
        with patch.object(bridge.rospy, 'get_param', side_effect=parameter), \
             patch.object(bridge.rospy, 'Subscriber'), patch.object(bridge.rospy, 'Timer'), \
             patch.object(bridge.socket, 'socket') as socket_factory:
            sender = bridge.ControlSender(Mock())
            socket_factory.return_value.bind.assert_called_once_with(('0.0.0.0', 9094))
            sender._timer_callback(None)
            socket_factory.return_value.sendto.assert_not_called()
            message = CtrlCmd(longlCmdType=1, accel=0.2, steering=0.0)
            sender._command_callback(message)
            sender._timer_callback(None)
            packet, target = socket_factory.return_value.sendto.call_args[0]
            self.assertEqual(target, ('192.168.0.1', 9093))
            payload = unpack_packet(packet)
            self.assertEqual(payload.type_name, 'MoraiCtrlCmd')
            self.assertEqual(struct.unpack('<BBB5f', payload.payload)[:3], (2, 4, 1))
            sender.close()

    def test_status_collision_and_sensor_channels_accept_only_simulator(self):
        node = Mock()
        node.simulator_ip = CONFIG['network']['simulator_ip']
        node.bind_ip = CONFIG['network']['bind_ip']
        node.receivers = []
        with patch.object(bridge.rospy, 'get_param', side_effect=parameter), \
             patch.object(bridge.rospy, 'loginfo'), \
             patch.object(bridge, 'UdpReceiver') as receiver, \
             patch.object(bridge, 'ControlSender') as sender:
            bridge.MoraiUdpBridge._configure_channels(node)
            endpoints = {call.args[0]: (call.args[2], call.args[4])
                         for call in receiver.call_args_list}
            self.assertEqual(endpoints, {
                'competition': (9099, '192.168.0.1'),
                'collision': (9092, '192.168.0.1'),
                'gps': (1234, '192.168.0.1'),
                'imu': (1236, '192.168.0.1'),
            })
            sender.assert_not_called()  # transport.launch explicitly enables it.


if __name__ == '__main__':
    unittest.main()
