# 대회 및 두 PC 테스트 통신 설정

근거: `catkin_ws/통신규정`의 2026-10-03 스크린샷. Client PC는 MORAI 시뮬 PC이며,
참가팀 PC는 ROS를 실행하는 PC다. 대회 시뮬 PC는 운영자가 설정한다.
우리 MORAI 테스트 PC에도 동일한 설정을 적용하면 대회와 같은 통신 환경을 사용한다.

## 유선 IPv4

| 항목 | MORAI 시뮬 PC (Client) | ROS 참가팀 PC |
|---|---|---|
| IP | 192.168.0.1 | 192.168.0.10 |
| 서브넷 마스크 | 255.255.255.0 (/24) | 255.255.255.0 (/24) |
| 게이트웨이 / DNS | 비움 | 비움 |
| 인터넷 기본 경로 | 이 유선 연결을 사용하지 않음 | 이 유선 연결을 사용하지 않음 |

Ubuntu 20.04 시뮬 PC에서 **설정 → 네트워크 → 시뮬 직결 유선의 톱니바퀴 → IPv4 → 수동**을
선택한다. 주소 192.168.0.1, 넷마스크 255.255.255.0을 입력하고 게이트웨이는 비운다.
DNS는 비워두고, Routes의 "이 연결에 있는 자원에만 사용"을 선택한다.
해당 유선 연결만 껐다 켜서 적용한다. Wi-Fi 전체를 끌 필요는 없다.
다른 인터넷용 랜에는 이 주소를 넣지 않는다.

이 ROS PC에서 사용한 인터페이스는 `enx588694f81ed5`, 활성 프로필 UUID는
`2afaf08e-7dcf-4760-9607-62d6479a296e`다. 같은 이름의 다른 morai-direct 프로필은 수정하지 않았다.
기존 192.168.100.1 설정은 자동 연결이 꺼진 `morai-direct-precompetition-20261003` 프로필에 보관한다.

## MORAI 네트워크 화면

Host IP는 **192.168.0.1**, Destination IP는 **192.168.0.10**, 전송 방식은 **UDP**다.
표의 Host/Destination은 MORAI 화면 기준이다. 특히 제어는 Host가 수신 포트다.

| 채널 | MORAI Host Port | MORAI Destination Port | 실제 방향 |
|---|---|---|---|
| Ego Ctrl Cmd | 9093 | 9094 | ROS 192.168.0.10:9094 → MORAI 192.168.0.1:9093 |
| Collision Data | 9091 | 9092 | MORAI → ROS 수신 9092 |
| Competition Vehicle Status | 9088 | 9099 | MORAI → ROS 수신 9099 |

센서 IP도 위와 같고, 센서 포트는 규정상 자유다. 아래는 현재 ROS와 맞는 설정이다.

| 센서 | 예시 MORAI Host Port (자유) | Destination Port (ROS 수신) |
|---|---|---|
| VLP-16 LiDAR | 2369 | 2368 |
| GPS | 1233 | 1234 |
| IMU | 1235 | 1236 |
| 전방 카메라 | 1231 | 1232 |
| 보조 카메라 | 1237 | 1238 |

Host Port는 자유지만 다른 채널과 중복되지 않게 선택한다. 각 채널을 Connect하고
기존 센서 위치·주기·해상도는 유지한다. 이 작업은 센서나 MPC 피팅 변경이 아니다.
기존 `apply_morai_udp_config.py`는 과거 포트와 카메라/라이다 활성화 정책을 사용하므로
대회 설정에 그대로 실행하지 않는다. 위 표대로 MORAI UI에서 설정한다.

## 실행 및 확인

ROS PC:

```bash
source ~/catkin_ws/devel/setup.bash
ping -c 3 192.168.0.1
roslaunch total_bringup competition.launch
```

시뮬 PC에서는 `ping -c 3 192.168.0.10`으로 반대 방향도 확인한다.
기본 런치는 시뮬 IP 192.168.0.1을 사용하며 UDP 제어를 켠다.
다른 시뮬 주소가 필요하면 `sim_ip:=주소`로 재정의할 수 있지만 포트는 별도 설정해야 한다.
`debug.launch`, `tuning_perception.launch`도 같은 대회 기본 IP/포트를 사용한다.

```bash
rostopic hz /Competition_topic
rostopic hz /gps
rostopic hz /imu
rostopic hz /velodyne_points
rostopic hz /image_jpeg/compressed
rostopic hz /image_jpeg2/compressed
```

충돌 정보는 이벤트 채널이므로 계속 들어오지 않아도 이상이 아니다.
`/CollisionData`는 실제 충돌 이벤트 또는 시뮬의 해당 채널 제공 시 확인한다.
시뮬 설정 완료 전에는 수신 여부를 확정할 수 없다. 설정 검증은 실제 제어 패킷 송신이나
백 재생 없이 수행한다. 런치 재시작은 차량을 정지시킨 뒤 수행한다.

## 복구

이 PC의 옛 유선 IP로 복구하려면 다음을 실행한다. Wi-Fi 설정은 바꾸지 않는다.

```bash
nmcli connection up uuid 472f6167-d438-4de4-a0ba-2d2f5f008790 ifname enx588694f81ed5
```

이 복구 프로필은 수동 연결용이다. ROS IP/포트와 시뮬 설정도 옛 환경에 맞춰야 하며,
소스 변경 전 백업은 `/tmp/morai_competition_before_20261003.tar.gz`에 있다.
Wi-Fi 절전 해제/실제 MAC 사용 설정은 KMU_WiFi 프로필에만 적용된 것으로,
대회장에 간다는 이유로 되돌릴 필요는 없다.
