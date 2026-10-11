# MORAI 원격 PC 연결 (LAN-to-LAN 직결)

MORAI 시뮬레이터를 다른 PC에서 실행하고, 이 PC에서 ROS 스택을 돌릴 때의 설정 절차.

```
[MORAI PC]  192.168.100.2  ←── 랜선 직결 ──→  192.168.100.1  [ROS PC]
   enp4s0                                      enx588694f81ed5 (USB 어댑터)
```

인터넷은 각 PC의 WiFi로 유지하고, 유선은 MORAI 전용으로만 쓴다.

---

## 0. 사전 확인 — 랜 포트가 살아 있는가

**이 단계를 건너뛰지 말 것.** 실제로 온보드 포트 고장 때문에 여기서 막혔다.

양쪽 PC에서 인터페이스 이름부터 확인한다.

```bash
ip -br link show
```

유선 인터페이스(`enp*`, `eno*`, USB 어댑터는 `enx*`)를 찾은 뒤:

```bash
sudo ip link set enx588694f81ed5 up
sleep 3
cat /sys/class/net/enx588694f81ed5/carrier
```

- `1` → 링크 정상. 1단계로.
- `0` → 아래 "문제 해결 / 링크가 안 잡힐 때" 참고.

랜선을 꽂았을 때 **포트 LED가 켜지는지**도 같이 본다. LED는 PHY 칩이 직접 켜는 것이라
OS·드라이버와 무관하다. 꺼져 있으면 물리 문제다.

---

## 1. IP 설정 (고정)

양쪽 모두 NetworkManager로 고정 IP 프로파일을 만든다.
**한 줄로 실행할 것** — 줄바꿈 `\` 뒤에 공백이 섞이면 파싱이 깨진다.

### ROS PC

```bash
sudo nmcli con add type ethernet ifname enx588694f81ed5 con-name morai-direct ipv4.method manual ipv4.addresses 192.168.100.1/24 ipv4.never-default yes ipv6.method ignore
sudo nmcli con up morai-direct
```

### MORAI PC

```bash
sudo nmcli con add type ethernet ifname enp4s0 con-name morai-direct ipv4.method manual ipv4.addresses 192.168.100.2/24 ipv4.never-default yes ipv6.method ignore
sudo nmcli con up morai-direct
```

`ipv4.never-default yes` 가 중요하다. 이게 없으면 인터넷 트래픽이 이 직결 링크로
가려다 끊긴다. 이 옵션 덕분에 기본 경로는 WiFi 로 남는다:

```bash
ip route | head -2
# default via 10.101.160.1 dev wlp4s0    <- 인터넷은 WiFi
```

### 확인

```bash
ip -br addr show                # 192.168.100.x 가 보여야 함
ping -c 3 192.168.100.2         # ROS PC 에서
```

정상이면 손실 0%, 지연 0.3 ms 안팎이 나온다.

---

## 2. 방화벽

양쪽 PC에서:

```bash
sudo ufw status
```

`inactive` 면 그대로 두면 된다. `active` 면:

```bash
sudo ufw allow from 192.168.100.0/24
```

---

## 3. MORAI Network Settings

각 센서의 udpConfig 에는 **host / destination 두 쌍**이 있다. 헷갈리기 쉬우니
역할을 먼저 확인한다.

```json
"hostIP": "0.0.0.0",           // MORAI PC 가 보낼 때 쓸 자기 인터페이스
"hostPort": 1233,              // MORAI 송신 포트
"destinationIP": "127.0.0.1",  // 받는 쪽 = ROS PC      ★ 여기만 바꾼다
"destinationPort": 1234        // ROS 수신 포트
```

| 필드                       | 값                  | 비고                           |
| -------------------------- | ------------------- | ------------------------------ |
| **destinationIP**          | **`192.168.100.1`** | ROS PC. **모든 센서에서 변경** |
| hostIP                     | `0.0.0.0`           | 그대로 (모든 인터페이스 사용)  |
| hostPort / destinationPort | —                   | 그대로                         |

`hostIP` 를 `192.168.100.2` 로 굳이 지정할 필요는 없다. `0.0.0.0` 이면
라우팅에 따라 알아서 유선으로 나간다.

### 포트 대응 (참고, 변경 불필요)

MORAI 의 `hostPort` 는 우리 수신 포트(`destinationPort`)보다 1 크다.
`morai_25s4.yaml` 의 `listen` 값이 `destinationPort` 와 일치한다.

| 채널               | hostPort (MORAI 송신) | destinationPort (ROS 수신) |
| ------------------ | --------------------- | -------------------------- |
| LiDAR (VLP-16)     | 2369                  | 2368                       |
| GPS                | 1233                  | 1234                       |
| IMU                | 1235                  | 1236                       |
| Camera (전방)      | 1231                  | 1232                       |
| Camera2 (신호등)   | 1239                  | 1238                       |
| Competition Status | —                     | 8000                       |
| Collision          | —                     | 8300                       |
| Ego Ctrl Cmd       | —                     | 7601 (**ROS → MORAI**)     |

`Ctrl Cmd` 만 방향이 반대다. ROS PC 가 MORAI PC 의 7601 로 보내며,
launch 의 `sim_ip` 인자로 자동 처리된다.

---

## 4. 패킷 수신 확인 (launch 전)

MORAI 를 실행한 뒤, ROS PC 에서:

```bash
sudo tcpdump -i enx588694f81ed5 -n udp -c 10
```

패킷이 보이면 MORAI 설정이 맞은 것이다. 안 보이면 destination IP 를 다시 확인한다.

채널별로 따로 보려면:

```bash
sudo tcpdump -i enx588694f81ed5 -n udp port 2368 -c 5   # LiDAR
sudo tcpdump -i enx588694f81ed5 -n udp port 1234 -c 5   # GPS
```

---

## 5. 실행

```bash
roslaunch total_bringup competition.launch sim_ip:=192.168.100.2
```

`sim_ip` 하나만 넘기면 UDP 브리지와 Velodyne 드라이버 양쪽에 전파된다
(`competition.launch` → `layers/transport.launch` → `morai_udp_bridge.launch`).

`config/morai_25s4.yaml` 의 `simulator_ip` 를 직접 고칠 필요는 없다.

### 토픽 확인

```bash
rostopic hz /velodyne_points /gps /imu /image_jpeg/compressed
```

기대값: LiDAR 10 Hz, 카메라 30 Hz.

---

## 대역폭

기가비트 링크면 여유가 충분하다.

| 채널                  | 대략           |
| --------------------- | -------------- |
| LiDAR (VLP-16, 10 Hz) | 8 Mbps         |
| 카메라 2대 (JPEG)     | 10 Mbps        |
| 기타                  | < 1 Mbps       |
| **합계**              | **약 20 Mbps** |

1000 Mbps 중 20 Mbps 만 쓰므로 USB 기가비트 어댑터로도 충분하다.
WiFi 로도 가능하지만 LiDAR 는 패킷 손실이 날 수 있으니,
WiFi 로 검증할 때는 `enable_lidar:=false` 로 시작하는 편이 낫다.

---

## 문제 해결 / 링크가 안 잡힐 때

`carrier=0`, LED 꺼짐, `Link detected: no` 인 경우의 점검 순서.

### 1) 양쪽 인터페이스가 모두 UP 인지

직결에서는 **양쪽 다 UP 이어야** carrier 가 잡힌다. 한쪽만 올려서는 안 된다.

### 2) 케이블 교차 검증

같은 케이블을 공유기에 꽂아 LED 가 켜지는지 본다.

- 켜짐 → 케이블 정상, 포트 쪽 문제
- 안 켜짐 → 케이블 불량 (육안으로 멀쩡해도 내부 단선은 흔하다)

### 3) 어느 쪽 포트가 문제인지 판별

각 PC 를 따로 공유기에 연결해 본다. LED 가 안 켜지는 쪽이 범인이다.

### 4) 드라이버 리로드

```bash
sudo modprobe -r igc && sleep 2 && sudo modprobe igc
sleep 3
sudo ethtool <iface> | grep -E "Supported link modes" -A3
```

`Supported link modes` 에 `1000baseT` / `2500baseT` 가 나와야 정상이다.
`10baseT` 만 나오면 PHY 초기화가 덜 된 상태다.

주의: `grep "Supported link modes"` 만 하면 **첫 줄만 출력**되어
`10baseT` 만 지원하는 것처럼 보인다. 반드시 `-A3` 를 붙일 것.

### 5) 콜드 부팅

Intel I225-V 는 warm reboot 으로 PHY 가 리셋되지 않는 사례가 있다.
**재부팅이 아니라 전원 차단**이어야 한다.

```
완전 종료 → 전원 케이블 분리 → 30초 → 재연결 → 부팅
```

### 6) BIOS

- Onboard LAN Controller: Enabled
- ErP / Deep Sleep 등 절전 옵션 끄기
- I225 관련 BIOS 업데이트 확인 (MSI 보드에서 수정 배포 이력 있음)

### 7) USB 이더넷 어댑터

위 방법이 모두 실패하면 온보드 포트 고장으로 본다.
USB 기가비트 어댑터로 우회하면 된다.

- `r8152`(Realtek) / `ax88179`(ASIX) 칩셋은 Ubuntu 20.04 에서 드라이버 없이 인식된다
- 꽂으면 `enx<MAC>` 형태의 인터페이스가 생긴다
- 성능은 온보드와 차이가 없다 (필요 대역폭이 20 Mbps 뿐)

```bash
dmesg | grep -iE "r8152|ax88|enx" | tail -5
ip -br link show
```

`carrier on` 이 보이면 성공. 1단계로 돌아가 IP 를 설정한다.

**USB 포트가 헐거우면 주행 중 끊긴다.** 단단히 꽂고 케이블이 당겨지지 않게 정리한다.

---

## 대회장 시나리오

연습 환경과 결정적으로 다른 점: **대회측 PC 설정을 우리가 못 바꾼다.**
특히 MORAI 의 destination IP 를 대회측이 우리 PC IP 로 넣어줘야 센서가 온다.
우리 쪽만 맞춰서는 데이터가 오지 않는다.

### 먼저 물어볼 것

1. IP 할당 방식 — 고정 지정인가, DHCP 인가
2. MORAI PC 의 IP
3. 우리가 쓸 IP (고정이면)
4. 포트 번호가 규정 기본값과 같은지
5. **destination IP 를 누가 설정하는가** (우리 IP 를 알려줘야 함)

우리 연습용 `192.168.100.x` 를 고집하지 말 것. 배정받은 대역을 쓴다.

### 인터페이스 이름 확인 (매번 먼저)

```bash
ip -br link show
```

USB 어댑터는 `enx<MAC>` 이라 PC/포트가 바뀌면 이름도 바뀐다.
아래 명령의 `<iface>` 를 여기서 확인한 이름으로 바꿔 쓴다.

### A. 고정 IP 를 배정받은 경우

```bash
sudo nmcli con add type ethernet ifname <iface> con-name morai-comp ipv4.method manual ipv4.addresses <배정받은IP>/24 ipv4.never-default yes ipv6.method ignore
sudo nmcli con up morai-comp
ip -br addr show <iface>
ping -c 3 <MORAI_PC_IP>
```

### B. DHCP 인 경우

```bash
sudo nmcli con add type ethernet ifname <iface> con-name morai-comp ipv4.method auto ipv4.never-default yes
sudo nmcli con up morai-comp
ip -br addr show <iface>
```

### C. 상대 IP 를 모를 때

MORAI 가 이미 패킷을 쏘고 있으면 송신지에서 바로 알 수 있다(가장 확실).

```bash
sudo tcpdump -i <iface> -n udp -c 5
```

출력의 `>` 왼쪽 IP 가 MORAI PC 다.

패킷이 없으면 같은 대역을 스캔한다.

```bash
sudo nmap -sn 192.168.x.0/24
```

`nmap` 이 없으면:

```bash
for i in $(seq 1 254); do ping -c1 -W1 192.168.x.$i &>/dev/null && echo "192.168.x.$i alive"; done
```

### 공통 — 연결 후

```bash
sudo ufw status
sudo ufw allow from <대역>/24          # active 일 때만
sudo tcpdump -i <iface> -n udp -c 10   # 패킷 수신 확인
roslaunch total_bringup competition.launch sim_ip:=<MORAI_PC_IP>
rostopic hz /velodyne_points /gps /imu /image_jpeg/compressed
```

### 안 될 때 빠른 점검

```bash
cat /sys/class/net/<iface>/carrier      # 1 이어야 함
ethtool <iface> | grep "Link detected"  # yes 여야 함
ip route | head -2                      # default 가 WiFi 로 가는지
sudo tcpdump -i <iface> -n udp -c 5     # 패킷이 오는지
```

- `carrier=0` → 물리 문제. 케이블/포트/어댑터 교체
- `carrier=1` 인데 패킷 없음 → **MORAI 쪽 destination IP 확인** (대회측에 요청)
- 패킷은 오는데 토픽 없음 → 포트 번호 불일치 또는 방화벽

### 준비물

- **USB 이더넷 어댑터** (온보드 포트 고장 상태 — 필수)
- **예비 USB 어댑터 1개** (현장에서 고장나면 대체 불가)
- 랜선 (대회측 제공 여부 확인)
- 이 문서

---

## 실제 사례 (2026-09-17)

- ROS PC 온보드 포트(`enp3s0`, Intel I225-V, MSI 보드)가 링크를 전혀 잡지 못했다.
  - `carrier_changes: 1` — 부팅 후 링크 이벤트 전무
  - LED 꺼짐, `Link detected: no`
  - 드라이버 리로드 / 콜드 부팅 / BIOS LAN 활성화 모두 효과 없음
  - 같은 케이블이 MORAI PC 에서는 LED 가 깜빡였으므로 케이블은 정상
  - → 온보드 포트 하드웨어 문제로 판단
- USB 이더넷 어댑터(`r8152`, `enx588694f81ed5`)로 우회 → 1000Mb/s Full 정상 연결
- `ping` 손실 0%, 지연 0.32 ms
