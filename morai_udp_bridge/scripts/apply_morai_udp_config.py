#!/usr/bin/env python3
"""Patch MORAI 25.S4 NetworkInfo/SensorInfo JSON files for this UDP bridge."""

import argparse
import datetime
import glob
import json
import shutil
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple


DEFAULT_ROOT = Path.home() / 'MoraiLauncher_Lin' / 'MoraiLauncher_Lin_Data' / 'SaveFile'


def find_latest(pattern: str) -> Path:
    matches = [
        Path(path)
        for path in glob.glob(pattern, recursive=True)
        if '.pre_udp_' not in Path(path).name
    ]
    if not matches:
        raise FileNotFoundError('no file matched {}'.format(pattern))
    return max(matches, key=lambda path: path.stat().st_mtime)


def set_udp(
    config: Dict,
    host_port: int,
    destination_port: int,
    host_ip: str,
    destination_ip: str,
) -> None:
    config['commType'] = 1
    udp = config.setdefault('udpConfig', {})
    udp.update({
        'hostIP': host_ip,
        'hostPort': host_port,
        'destinationIP': destination_ip,
        'destinationPort': destination_port,
        'sizeOfAuxHeader': 12,
    })
    ros = config.get('rosConfig')
    if isinstance(ros, dict):
        ros['RosMessageOnOff'] = False


def disable_config(config: Optional[Dict]) -> None:
    if not isinstance(config, dict):
        return
    config['commType'] = 0
    ros = config.get('rosConfig')
    if isinstance(ros, dict):
        ros['RosMessageOnOff'] = False


def set_rosbridge(config: Dict, topic: str, rosbridge_ip: str, rosbridge_port: int = 9090) -> None:
    config['commType'] = 3
    ros = config.setdefault('rosConfig', {})
    ros.update({
        'handlerType': 1,
        'RosBridgeServerUrl': 'ws://{}:{}'.format(rosbridge_ip, rosbridge_port),
        'RosBridgeIP': rosbridge_ip,
        'RosBridgePort': rosbridge_port,
        'Topic': topic,
        'RosMessageOnOff': True,
    })


def network_entries(document: Dict) -> Iterable[Dict]:
    for ego in document.get('egoNetworkData', []):
        yield from ego.get('listEgoNetworkInfo', [])


def patch_network(
    document: Dict,
    keep_unlisted: bool,
    host_ip: str,
    destination_ip: str,
    ctrl_rosbridge_port: int,
    ctrl_transport: str = 'udp',
    ctrl_host_port: int = 7601,
    ctrl_destination_port: int = 7600,
) -> None:
    found = set()
    for entry in network_entries(document):
        pubsub = entry.get('PUBSUB_TYPE')
        if pubsub == 771:  # Ego Ctrl Cmd
            # 규정 2-8의 허용 네트워크는 UDP 7종뿐이다. ROS Bridge 는 목록에 없으므로
            # 기본값을 UDP 로 둔다. hostPort 는 MORAI 가 수신하는 포트이며
            # morai_udp_bridge 의 ports/control/simulator 와 같아야 한다.
            if ctrl_transport == 'rosbridge':
                set_rosbridge(entry['EgoCtrlConfig'], '/ctrl_cmd',
                              destination_ip, ctrl_rosbridge_port)
            else:
                set_udp(entry['EgoCtrlConfig'], ctrl_host_port,
                        ctrl_destination_port, host_ip, destination_ip)
            disable_config(entry.get('config'))
            found.add(pubsub)
        elif pubsub == 261:  # Collision Data
            set_udp(entry['config'], 8301, 8300, host_ip, destination_ip)
            found.add(pubsub)
        elif pubsub == 295:  # Competition Vehicle Status
            set_udp(entry['config'], 8001, 8000, host_ip, destination_ip)
            found.add(pubsub)
        elif not keep_unlisted:
            disable_config(entry.get('config'))
            disable_config(entry.get('EgoCtrlConfig'))
    missing = {771, 261, 295} - found
    if missing:
        raise ValueError('required PUBSUB_TYPE entries missing: {}'.format(sorted(missing)))


def sensor_config(entry: Dict) -> Tuple[str, Dict]:
    for key, value in entry.items():
        if isinstance(value, dict) and 'commType' in value and 'udpConfig' in value:
            return key, value
    raise ValueError('sensor entry has no communication config')


def ros_topic(entry: Dict) -> str:
    try:
        _, config = sensor_config(entry)
    except ValueError:
        return ''
    return str(config.get('rosConfig', {}).get('Topic', ''))


def activate_first(
    entries,
    host_port: int,
    destination_port: int,
    host_ip: str,
    destination_ip: str,
) -> None:
    for index, entry in enumerate(entries):
        _, config = sensor_config(entry)
        if index == 0:
            set_udp(config, host_port, destination_port, host_ip, destination_ip)
        else:
            disable_config(config)


def activate_all(entries, first_host_port: int, first_destination_port: int,
                 host_ip: str, destination_ip: str) -> None:
    """Give every same-type high-bandwidth sensor its own UDP port pair."""
    for index, entry in enumerate(entries):
        _, config = sensor_config(entry)
        set_udp(config, first_host_port + 2 * index,
                first_destination_port + 2 * index,
                host_ip, destination_ip)


def patch_sensors(document: Dict, host_ip: str, destination_ip: str) -> None:
    activate_all(document.get('Lidar3DList', []), 2369, 2368, host_ip, destination_ip)
    activate_first(document.get('IMUList', []), 1235, 1236, host_ip, destination_ip)
    activate_first(document.get('cameraList', []), 1231, 1232, host_ip, destination_ip)

    gps_entries = document.get('GPSList', [])
    selected = next((item for item in gps_entries if ros_topic(item) == '/gps'), None)
    if selected is None and gps_entries:
        selected = gps_entries[0]
    for entry in gps_entries:
        _, config = sensor_config(entry)
        if entry is selected:
            set_udp(config, 1233, 1234, host_ip, destination_ip)
        else:
            disable_config(config)


def write_with_backup(path: Path, document: Dict, no_backup: bool) -> Optional[Path]:
    backup = None
    if not no_backup:
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        backup = path.with_name('{}{}.json'.format(path.stem, '.pre_udp_' + stamp))
        shutil.copy2(str(path), str(backup))
    with path.open('w', encoding='utf-8') as output:
        json.dump(document, output, indent=2, ensure_ascii=False)
        output.write('\n')
    return backup


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--network-file', type=Path)
    parser.add_argument('--sensor-file', type=Path)
    parser.add_argument(
        '--keep-unlisted-network',
        action='store_true',
        help='do not disable TF, Ego/Object status, services, and other ROS channels',
    )
    parser.add_argument('--no-backup', action='store_true')
    parser.add_argument(
        '--dry-run', action='store_true',
        help='validate and print the selected files without writing or backing them up',
    )
    parser.add_argument(
        '--host-ip',
        default='0.0.0.0',
        help='local IP this MORAI instance binds its UDP sockets to '
             '(default 0.0.0.0 = all interfaces; works for same-PC and cross-PC)',
    )
    parser.add_argument(
        '--destination-ip',
        default='127.0.0.1',
        help='IP of the machine running morai_udp_bridge (ROS side). '
             'Keep 127.0.0.1 when MORAI and ROS run on the same PC; '
             'set to the ROS PC\'s LAN IP when MORAI runs on a different PC',
    )
    parser.add_argument(
        '--ctrl-transport',
        choices=['udp', 'rosbridge'],
        default='udp',
        help='Ego Ctrl Cmd 전송 방식. 규정 2-8은 UDP만 허용하므로 기본 udp. '
             'rosbridge 는 대회장 문제 시 폴백용',
    )
    parser.add_argument(
        '--ctrl-host-port', type=int, default=7601,
        help='MORAI 가 Ctrl Cmd 를 수신할 포트 '
             '(morai_udp_bridge 의 ports/control/simulator 와 일치해야 함)',
    )
    parser.add_argument(
        '--ctrl-destination-port', type=int, default=7600,
        help='ROS 쪽 Ctrl Cmd 포트 (ports/control/local)',
    )
    parser.add_argument(
        '--ctrl-rosbridge-port',
        type=int,
        default=9090,
        help='rosbridge_websocket port used only for /ctrl_cmd (default 9090)',
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    network_path = args.network_file or find_latest(
        str(DEFAULT_ROOT / 'Network' / '**' / 'NetworkInfo_*.json')
    )
    sensor_path = args.sensor_file or find_latest(
        str(DEFAULT_ROOT / 'Sensor' / '**' / 'SensorInfo_*.json')
    )
    network = json.loads(network_path.read_text(encoding='utf-8'))
    sensors = json.loads(sensor_path.read_text(encoding='utf-8'))
    patch_network(
        network,
        args.keep_unlisted_network,
        args.host_ip,
        args.destination_ip,
        args.ctrl_rosbridge_port,
        args.ctrl_transport,
        args.ctrl_host_port,
        args.ctrl_destination_port,
    )
    patch_sensors(sensors, args.host_ip, args.destination_ip)
    network_backup = sensor_backup = None
    if not args.dry_run:
        network_backup = write_with_backup(network_path, network, args.no_backup)
        sensor_backup = write_with_backup(sensor_path, sensors, args.no_backup)
    print('MORAI UDP profile updated:')
    print('  network: {}'.format(network_path))
    print('  sensors: {}'.format(sensor_path))
    print('  MORAI host_ip: {}  ->  ROS destination_ip: {}'.format(args.host_ip, args.destination_ip))
    if args.ctrl_transport == 'rosbridge':
        print('  ctrl_cmd ROSBridge: ws://{}:{}'.format(
            args.destination_ip, args.ctrl_rosbridge_port))
    else:
        print('  ctrl_cmd UDP: MORAI hostPort {} <- ROS {}'.format(
            args.ctrl_host_port, args.ctrl_destination_port))
    if args.dry_run:
        print('  DRY RUN: no files or backups were written')
    if network_backup and sensor_backup:
        print('  backups: {}, {}'.format(network_backup, sensor_backup))
    print('Restart MORAI SIM before launching the bridge.')


if __name__ == '__main__':
    main()
