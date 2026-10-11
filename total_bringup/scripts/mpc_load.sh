#!/usr/bin/env bash
# mpc_path_tracking 의 CPU 점유와 명령 주기를 함께 잰다.
#
# 판정
#   CPU 95~100%  -> 코어 하나를 꽉 씀. IPOPT 계산 부하가 원인.
#   CPU 40~60%   -> 계산이 아니라 대기. 콜백 큐/입력 타이밍을 봐야 함.
#   CPU 10% 이하 -> 루프가 블로킹됨. 별도 조사.
#
# 사용: rosrun total_bringup mpc_load.sh [측정초]
set -u
SECS="${1:-6}"

# Linux comm 은 15자로 잘린다(mpc_path_tracking -> mpc_path_tracki).
# 전체 커맨드라인으로 찾는 편이 확실하다.
PID=$(pgrep -f "lib/mpc_path_tracking/mpc_path_tracking$" | head -1)
[ -z "$PID" ] && PID=$(pgrep -x mpc_path_tracki | head -1)

if [ -z "$PID" ] || [ ! -d "/proc/$PID" ]; then
  echo "mpc_path_tracking 프로세스를 찾지 못했습니다. 스택이 떠 있는지 확인하세요."
  exit 1
fi

CLK=$(getconf CLK_TCK)
cpu_ticks() { awk '{print $14+$15}' "/proc/$PID/stat"; }

echo "PID $PID  스레드 $(ls /proc/$PID/task | wc -l)  코어 $(nproc)"
echo "$(uptime | grep -oP 'load average.*')"
echo "--- CPU (1초 간격 ${SECS}회) ---"

prev=$(cpu_ticks)
sum=0
for _ in $(seq 1 "$SECS"); do
  sleep 1
  [ -d "/proc/$PID" ] || { echo "  프로세스 종료됨"; break; }
  cur=$(cpu_ticks)
  pct=$(awk -v a="$cur" -v b="$prev" -v c="$CLK" 'BEGIN{printf "%.1f",(a-b)*100/c}')
  echo "  ${pct}%"
  sum=$(awk -v s="$sum" -v p="$pct" 'BEGIN{print s+p}')
  prev=$cur
done
echo "평균 $(awk -v s="$sum" -v n="$SECS" 'BEGIN{printf "%.1f", s/n}')%"

echo "--- 명령 주기 ---"
if command -v rostopic >/dev/null 2>&1; then
  timeout $((SECS + 4)) rostopic hz /control/raw_cmd 2>/dev/null \
    | grep -m1 "average rate" || echo "  /control/raw_cmd 측정 실패"
fi
