"""차동구동 운동학: cmd_vel(v, w) -> 좌/우 바퀴 각속도.

시뮬레이션 노드(pybullet_sim)와 실물 시리얼 노드(serial_node)가
이 파일을 똑같이 사용합니다. 시뮬레이션에서 검증한 계산이 그대로 실물에 들어갑니다.

바퀴 치수는 config/robot_layout.yaml 에서 정하고,
tools/build_robot.py 가 robot_params.py 로 자동 생성합니다.
"""
from robot_car.robot_params import MAX_WHEEL_SPEED, WHEEL_RADIUS, WHEEL_SEPARATION  # noqa: F401


def twist_to_wheels(v, w):
    """선속도 v[m/s], 각속도 w[rad/s] -> (왼쪽, 오른쪽) 바퀴 각속도 [rad/s].

    한쪽 바퀴가 최대 속도를 넘으면 두 바퀴를 같은 비율로 줄여서
    회전 반경(진행 방향)은 유지합니다.
    """
    wl = (v - w * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
    wr = (v + w * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS

    biggest = max(abs(wl), abs(wr))
    if biggest > MAX_WHEEL_SPEED:
        scale = MAX_WHEEL_SPEED / biggest
        wl *= scale
        wr *= scale
    return wl, wr


def wheel_speed_to_pwm(wheel_speed):
    """바퀴 각속도 [rad/s] -> 아두이노 PWM(-255~255). 실제 시리얼 노드에서 사용."""
    pwm = int(round(wheel_speed / MAX_WHEEL_SPEED * 255))
    return max(-255, min(255, pwm))
