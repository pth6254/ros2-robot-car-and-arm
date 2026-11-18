"""ESP32 상태 보고 -> 알기 쉬운 로그. tcp_node(WiFi) 와 serial_node(USB) 가 공통으로 사용.

    "BOOT,원인,켜진초"                  접속할 때 한 번. 마지막 재부팅 원인과 켜진 지 몇 초인지
    "STAT,워치독횟수,버린보고,켜진초"    5초마다. 늘어났을 때만 경고

재부팅 원인
    BROWNOUT : 전압 부족. 급회전/급가속 때 배터리 전압이 떨어져 ESP32 가 꺼졌다 켜짐
    POWERON  : 전원을 켬, USB 연결, EN 버튼, 시리얼 포트 열기
    PANIC/WDT: 프로그램 오류나 멈춤
"""

REASON_TEXT = {
    'BROWNOUT': '전압 부족 -> 배터리 충전 상태, 급회전/급가속, ESP32 VIN-GND 콘덴서(100~470uF) 확인',
    'POWERON': '전원 켜짐 (배터리 스위치, USB 연결, EN 버튼, 시리얼 포트 열기)',
    'SW': '프로그램 재시작 (업로드 직후 등)',
    'PANIC': '프로그램 오류로 재시작',
    'WDT': '프로그램이 멈춰서 감시 타이머가 재시작',
}
RECENT_BOOT_S = 30   # 켜진 지 이 시간 이내면 '방금 재부팅됨'으로 경고


class BoardStatus:
    def __init__(self, node):
        self.node = node
        self.failsafe = None
        self.drops = None

    def handle_line(self, text):
        """BOOT/STAT 줄이면 처리하고 True."""
        log = self.node.get_logger()
        if text.startswith('BOOT,'):
            try:
                _, reason, up = text.split(',')[:3]
                up = int(up)
            except ValueError:
                return True
            desc = REASON_TEXT.get(reason, reason)
            # 방금 재부팅됐고, 사람이 전원을 켠 것(POWERON)이 아니면 경고 (전압 부족, 오류 등)
            if up < RECENT_BOOT_S and reason != 'POWERON':
                log.warn(f'ESP32 가 {up}초 전에 재부팅됨. 원인: {reason} ({desc})')
            else:
                log.info(f'ESP32 켜진 지 {up}초. 마지막 재부팅 원인: {reason} ({desc})')
            return True
        if text.startswith('STAT,'):
            try:
                failsafe, drops = (int(v) for v in text.split(',')[1:3])
            except ValueError:
                return True
            if self.failsafe is not None and failsafe > self.failsafe:
                log.warn(f'ESP32 독립 워치독이 모터를 정지시킴 (+{failsafe - self.failsafe}회): '
                         'ESP32 프로그램이 잠시 멈췄거나 명령이 끊김')
            if self.drops is not None and drops > self.drops:
                log.warn(f'WiFi 전송이 막혀 ESP32 상태 보고 {drops - self.drops}개를 버림 (신호 약함)',
                         throttle_duration_sec=10.0)
            self.failsafe, self.drops = failsafe, drops
            return True
        return False
