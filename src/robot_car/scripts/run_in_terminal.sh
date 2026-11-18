#!/usr/bin/env bash
# 사용: run_in_terminal.sh <창 제목> <명령...>
#
# 키보드 입력이 필요한 노드(wasd_teleop)를 새 gnome-terminal 창에서 실행하고,
# launch 가 종료되면(Ctrl+C) 창 속 프로그램도 함께 종료시킨다.
#
# 필요한 이유: gnome-terminal 은 창을 '터미널 서버'에 요청만 하고 끝나기 때문에,
# launch 가 gnome-terminal 을 종료해도 창 속 프로그램에는 신호가 가지 않는다.

title="$1"; shift
pidfile="$(mktemp)"

# 새 창에서: 자기 PID 를 파일에 적고 -> exec 로 명령 실행 (PID 그대로 유지)
gnome-terminal --title="$title" -- bash -c 'echo $$ > "$1"; shift; exec "$@"' _ "$pidfile" "$@"

# 창 속 프로그램 PID 기다리기 (최대 10초)
for _ in $(seq 100); do [ -s "$pidfile" ] && break; sleep 0.1; done
pid="$(cat "$pidfile" 2>/dev/null)"; rm -f "$pidfile"
if [ -z "$pid" ]; then
    echo "[run_in_terminal] 터미널 창을 열지 못했습니다" >&2
    exit 1
fi

stop_child() {
    kill -INT "$pid" 2>/dev/null                       # 정상 종료 요청
    for _ in $(seq 30); do kill -0 "$pid" 2>/dev/null || exit 0; sleep 0.1; done
    kill -TERM "$pid" 2>/dev/null                      # 3초 안에 안 끝나면 강제
    exit 0
}
trap stop_child INT TERM

# 창 속 프로그램이 살아 있는 동안 대기 (사용자가 창을 닫으면 여기서도 종료)
while kill -0 "$pid" 2>/dev/null; do
    sleep 0.5 & wait $!
done
