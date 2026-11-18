"""ESP32 가 보내는 IR 장애물 센서 보고("IR,왼쪽,오른쪽") -> ROS 토픽.

tcp_node(WiFi) 와 serial_node(USB) 가 공통으로 사용합니다.

    /ir/left, /ir/right  (sensor_msgs/Range, frame_id = ir_left / ir_right)
        장애물 감지 시  range = max_range  -> RViz 에 감지 구역(원뿔)이 보임
        없음           range = +inf       -> RViz 에 아무것도 안 보임

FC-51 같은 IR 모듈은 거리를 재지 않고 '감지됨/안 됨'만 알려주므로,
range 값은 거리가 아니라 감지 여부를 표시하는 용도입니다.
실제 '장애물 앞에서 전진 정지'는 ESP32 안에서 바로 처리합니다 (WiFi 지연과 무관).
"""
import math

from sensor_msgs.msg import Range

IR_MAX_RANGE = 0.30        # [m] FC-51 감지 거리 약 2~30 cm (가변저항으로 조절)
IR_MIN_RANGE = 0.02
IR_FOV = math.radians(35)  # 감지 각도 (대략)


class IrReporter:
    def __init__(self, node):
        self.node = node
        self.pubs = {side: node.create_publisher(Range, f'ir/{side}', 10) for side in ('left', 'right')}
        self.state = None

    def handle_line(self, text):
        """'IR,1,0' 형식이면 발행하고 True, 아니면 False (다른 메시지는 호출한 쪽에서 처리)."""
        if not text.startswith('IR,'):
            return False
        try:
            left, right = (int(v) == 1 for v in text.split(',')[1:3])
        except ValueError:
            return True
        stamp = self.node.get_clock().now().to_msg()
        for side, hit in (('left', left), ('right', right)):
            msg = Range()
            msg.header.stamp = stamp
            msg.header.frame_id = f'ir_{side}'
            msg.radiation_type = Range.INFRARED
            msg.field_of_view = IR_FOV
            msg.min_range = IR_MIN_RANGE
            msg.max_range = IR_MAX_RANGE
            msg.range = IR_MAX_RANGE if hit else math.inf
            self.pubs[side].publish(msg)
        if (left, right) != self.state:      # 바뀔 때만 로그
            self.state = (left, right)
            sides = [n for n, h in (('왼쪽', left), ('오른쪽', right)) if h]
            if sides:
                self.node.get_logger().warn(f'IR 장애물 감지: {", ".join(sides)} -> ESP32 가 전진 차단')
            else:
                self.node.get_logger().info('IR 장애물 없음')
        return True
