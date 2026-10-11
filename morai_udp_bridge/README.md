# MORAI UDP Bridge

현재 대회 기본값은 시뮬 PC `192.168.0.1`, 참가팀 PC `192.168.0.10`이며
대회 transport는 `/ctrl_cmd`도 UDP로 전송합니다. 두 Ubuntu PC의 IP·포트 설정은
[대회 통신 설정](docs/competition_network.md)을 따르세요.
아래 ROS Bridge 및 자동 JSON 변환 안내는 과거 환경용이며 대회 기본 설정과 다릅니다.

MORAI SIM 25.S4와 ROS1 사이의 통신 경계를 채널별로 UDP 또는 ROS Bridge로
바꾸는 패키지다. 기존 제어기와 센서 처리 노드는 계속 ROS 토픽을 사용하므로
제어 코드를 수정할 필요가 없다.

`/ctrl_cmd`만 ROS Bridge(`rosbridge_websocket`)로 나가고, 나머지 고빈도/대역폭
채널(센서·경쟁상태)은 UDP를 쓴다. ROS Bridge는 이미 `/ctrl_cmd`를 발행하는
컨트롤러가 있으므로 타입 등록 없이 그대로 중계된다.

## ROS 토픽 연결

| 방향 | 전송 | ROS 토픽 |
|---|---|---|
| ROS -> MORAI | ROS Bridge (`rosbridge_websocket`, port 9090) | `/ctrl_cmd` |
| MORAI -> ROS | UDP | `/Competition_topic` |
| MORAI -> ROS | UDP | `/CollisionData` |
| MORAI -> ROS | UDP | `/gps` |
| MORAI -> ROS | UDP | `/imu` |
| MORAI -> ROS | UDP | `/image_jpeg/compressed` |
| MORAI -> ROS | UDP | `/velodyne_points` |

Competition UDP 데이터에 포함된 현재 HD map link ID는
`/morai/current_link_id`에도 발행한다.

## 한 번만 하는 MORAI 설정

MORAI를 종료한 상태에서 현재 25.S4 저장 프로필을 자동 변환한다. 원본 옆에
`pre_udp_날짜` 백업 파일이 먼저 만들어진다.

이 스크립트는 표준 라이브러리만 쓰므로 ROS가 없는 MORAI 전용 PC에서도
`apply_morai_udp_config.py` 파일 하나만 복사해 `python3`로 바로 실행할 수 있다.

`--destination-ip`는 UDP 목적지뿐 아니라 Ctrl Cmd의 `RosBridgeServerUrl`
(`ws://<destination-ip>:9090`)에도 쓰인다 - 두 채널 모두 결국 같은 ROS PC를
가리켜야 하기 때문이다. rosbridge 포트를 바꾸면
`--ctrl-rosbridge-port`와 `morai_ctrl_rosbridge.launch`의 `port`를 같은 값으로
맞춘다.

**MORAI와 ROS(브리지)가 같은 PC** (기본값, 아무 옵션 없이 실행):

```bash
source ~/catkin_ws/devel/setup.bash
rosrun morai_udp_bridge apply_morai_udp_config.py
```

**MORAI와 ROS가 서로 다른 PC**: MORAI가 도는 PC에서 실행하되, `--destination-ip`에
ROS(브리지)가 도는 PC의 LAN IP를 지정한다.

```bash
python3 apply_morai_udp_config.py --destination-ip 192.168.0.42
```

그리고 ROS 쪽에서는 `roslaunch` 인자로 MORAI PC의 IP를 알려준다 (아래 "빌드와 실행"
참고). 두 PC 모두 관련 포트가 방화벽에 막히지 않아야 한다 (Ctrl Cmd 9090은 ROS
Bridge, 나머지 센서/경쟁상태 UDP 포트는 ROS 쪽에서 인바운드 허용).

변환 결과의 핵심 포트는 다음과 같다.

| 채널 | 전송 | MORAI hostPort | MORAI destinationPort |
|---|---|---:|---:|
| Ctrl Cmd | ROS Bridge | - | 9090 (ws, `--ctrl-rosbridge-port`) |
| Competition Status | UDP | 8001 | 8000 |
| Collision | UDP | 8301 | 8300 |
| Camera | UDP | 1231 | 1232 |
| GPS | UDP | 1233 | 1234 |
| IMU | UDP | 1235 | 1236 |
| VLP-16 #1 | UDP | 2369 | 2368 |
| VLP-16 #2 | UDP | 2371 | 2370 |

3D LiDAR가 여러 개면 위 포트에서 센서 인덱스마다 2씩 증가시켜 각 드라이버에
고유 포트를 배정한다. 현재 센서 프로필에 GPS가 두 개 있으면 ROS 토픽이 `/gps`였던 센서 하나만 UDP로
활성화한다. 카메라는 SensorInfo에 카메라가 실제로 추가되어 있을 때만 설정된다.

## GPS 중복 datagram 제거

MORAI 25.S4는 GPS 한 fix를 **RMC와 GGA 두 datagram으로 따로** 보낸다. 두 패킷은
1ms 미만 간격으로 도착하고 lat/lon이 완전히 같다(GGA에만 고도가 있다). 둘 다
발행하면 20Hz 센서가 37Hz처럼 보이고, EKF가 **같은 측정을 독립 관측 2회로
반영해 위치를 과신**한다 (실효 `sigma_gps`가 1/√2로 축소).

2026-08-11 주행로그 3건에서 `/gps` 메시지의 정확히 50%가 이 중복이었다. 기본값
(`gps/dedupe_duplicate_sentence: true`)은 GGA를 한 번 받은 뒤부터 RMC만 든
datagram을 버려 원래 20Hz를 복원한다. GGA가 없는 프로필에서는 RMC를 그대로
쓰므로 안전하다.

## IMU timestamp 중복 제거

25.S4 IMU UDP payload에는 MORAI 원본 `sec/nsec`가 들어 있다. 브리지는 이 값을
`/imu.header.stamp`로 보존하고, `imu/drop_non_increasing_stamp: true`일 때 같은
timestamp가 반복되거나 `imu/min_stamp_step`보다 가까운 샘플은 버린다. 이렇게 해야
UDP/ROS 변환 과정에서 생긴 burst가 EKF에 별도 샘플처럼 두 번 들어가지 않는다.

## 빌드와 실행

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# 터미널 1: UDP 센서 브리지 + VLP-16 드라이버
roslaunch morai_udp_bridge morai_udp_all.launch

# 터미널 2: ctrl_cmd 전용 ROSBridge
roslaunch morai_udp_bridge morai_ctrl_rosbridge.launch

# 터미널 3: 주행 스택 (통신 브리지를 내부에서 실행하지 않음)
roslaunch morai_mgeo mgeo_mpc_drive.launch
```

두 브리지는 의도적으로 서로 include하지 않으며 주행 launch에도 포함되지 않는다.
`morai_ctrl_rosbridge.launch`의 websocket은 `/ctrl_cmd`만 허용하고
service/parameter와 rosapi는 노출하지 않는다. `morai_udp_all.launch`는 UDP control
sender를 항상 끄므로 `/ctrl_cmd`를 UDP로 동시에 보내지 않는다.

카메라 센서를 추가했다면 다음처럼 켠다.

```bash
roslaunch morai_udp_bridge morai_udp_all.launch enable_camera:=true
```

MORAI가 다른 PC에서 돈다면 `simulator_ip`에 그 PC의 IP를 준다 (위
`apply_morai_udp_config.py --destination-ip`에 준 이 ROS PC의 IP와 짝을 이룸):

```bash
roslaunch morai_udp_bridge morai_udp_all.launch simulator_ip:=192.168.0.10
```

## 확인

```bash
rostopic hz /Competition_topic
rostopic hz /gps
rostopic hz /imu
rostopic hz /lidar3d_0
rostopic hz /lidar3d_1
rostopic echo -n 1 /diagnostics

# ctrl_cmd ROS Bridge 연결 확인 (MORAI가 클라이언트로 붙으면 subscriber 수가 늘어난다)
rostopic info /ctrl_cmd
```

`/ctrl_cmd`는 ROS Bridge를 통해 MORAI로 전달되므로, UDP 채널에 있던 0.3초
정지 워치독(`brake=0.7` 안전 명령)은 이 경로에는 적용되지 않는다 - 제어기
자체의 정지/폴백 로직에 의존한다.

## UDP ctrl_cmd로 되돌리기 (선택)

ROS Bridge 대신 UDP로 다시 보내려면 `config/morai_25s4.yaml`의
`enable.control`을 `true`로 되돌리고, MORAI NetworkInfo의 Ego Ctrl Cmd 항목도
`apply_morai_udp_config.py`의 이전 UDP 설정(`set_udp`)으로 되돌려야 한다. UDP
경로에서는 `CtrlCmd.steering`(rad, 전륜각)을 최대 전륜각 기준 `-1~1` 값으로
정규화해서 보낸다.

```text
udp_steer = clamp(ros_steering_rad / radians(40), -1, 1)
```

그리고 0.3초 이상 명령이 끊기면 `brake=0.7` 안전 명령을 대신 보낸다
(`control.command_timeout`, `control.stale_brake`).
