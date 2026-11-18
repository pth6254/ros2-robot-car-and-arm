# 새 터미널을 열 때마다 (프로젝트 폴더에서):  source env.sh
# (ROS2 + venv + 워크스페이스를 한 번에 활성화)
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$WS/venv/bin/activate"
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"

# colcon 을 venv 의 python 으로 실행 -> 노드 실행 스크립트가 venv python 을 쓰게 됨
# (/usr/bin/colcon 을 그대로 쓰면 shebang 이 /usr/bin/python3 로 박혀서 pybullet 을 못 찾음)
colcon() { python3 -m colcon "$@"; }

echo "[env] ROS2 jazzy + venv($(python3 -c 'import sys;print(sys.prefix)')) 활성화"
