#!/usr/bin/env python3
import importlib.util
import pathlib
import unittest


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / 'scripts' / 'apply_morai_udp_config.py'
SPEC = importlib.util.spec_from_file_location('apply_morai_udp_config', str(SCRIPT))
apply_config = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(apply_config)


class ApplyMoraiUdpConfigTest(unittest.TestCase):
    def test_ctrl_cmd_is_rosbridge_and_status_channels_are_udp(self):
        document = {
            'egoNetworkData': [{
                'listEgoNetworkInfo': [
                    {
                        'PUBSUB_TYPE': 771,
                        'EgoCtrlConfig': {
                            'commType': 0,
                            'rosConfig': {},
                            'udpConfig': {},
                        },
                        'config': {'commType': 1, 'rosConfig': {'RosMessageOnOff': True}},
                    },
                    {
                        'PUBSUB_TYPE': 261,
                        'config': {'commType': 0, 'udpConfig': {}, 'rosConfig': {}},
                    },
                    {
                        'PUBSUB_TYPE': 295,
                        'config': {'commType': 0, 'udpConfig': {}, 'rosConfig': {}},
                    },
                ],
            }],
        }

        apply_config.patch_network(
            document,
            keep_unlisted=False,
            host_ip='0.0.0.0',
            destination_ip='127.0.0.1',
            ctrl_rosbridge_port=9090,
            ctrl_transport='rosbridge',
        )

        ctrl_entry = document['egoNetworkData'][0]['listEgoNetworkInfo'][0]
        ctrl_config = ctrl_entry['EgoCtrlConfig']
        self.assertEqual(ctrl_config['commType'], 3)
        self.assertEqual(ctrl_config['rosConfig']['Topic'], '/ctrl_cmd')
        self.assertEqual(ctrl_config['rosConfig']['RosBridgeServerUrl'], 'ws://127.0.0.1:9090')
        self.assertFalse(ctrl_entry['config']['rosConfig']['RosMessageOnOff'])

        collision_config = document['egoNetworkData'][0]['listEgoNetworkInfo'][1]['config']
        competition_config = document['egoNetworkData'][0]['listEgoNetworkInfo'][2]['config']
        self.assertEqual(collision_config['commType'], 1)
        self.assertEqual(collision_config['udpConfig']['destinationPort'], 8300)
        self.assertEqual(competition_config['commType'], 1)
        self.assertEqual(competition_config['udpConfig']['destinationPort'], 8000)

    def test_each_lidar_gets_a_unique_udp_port(self):
        def sensor(topic):
            return {'LidarConfig': {
                'commType': 0, 'udpConfig': {},
                'rosConfig': {'Topic': topic, 'RosMessageOnOff': True}}}
        document = {
            'Lidar3DList': [sensor('/lidar3d_0'), sensor('/lidar3d_1')],
            'IMUList': [], 'cameraList': [], 'GPSList': [],
        }
        apply_config.patch_sensors(document, '0.0.0.0', '127.0.0.1')
        first = document['Lidar3DList'][0]['LidarConfig']['udpConfig']
        second = document['Lidar3DList'][1]['LidarConfig']['udpConfig']
        self.assertEqual((first['hostPort'], first['destinationPort']), (2369, 2368))
        self.assertEqual((second['hostPort'], second['destinationPort']), (2371, 2370))


if __name__ == '__main__':
    unittest.main()
