#!/usr/bin/env python3
"""robot_layout.yaml -> URDF 생성 + 배치 검사 + PyBullet 주행 테스트 + RViz 갱신.

    source env.sh
    python3 tools/build_robot.py            # 전부 실행
    python3 tools/build_robot.py --no-sim   # 주행 테스트 생략
    python3 tools/build_robot.py --no-reload

생성 파일 (직접 수정 금지):
    src/robot_car/urdf/robot_car.urdf
    src/robot_car/robot_car/robot_params.py   (kinematics.py 가 사용)
"""
import argparse
import math
import os
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, 'src', 'robot_car')
LAYOUT = os.path.join(PKG, 'config', 'robot_layout.yaml')
URDF_OUT = os.path.join(PKG, 'urdf', 'robot_car.urdf')
PARAMS_OUT = os.path.join(PKG, 'robot_car', 'robot_params.py')

COLORS = {
    'white': '0.92 0.92 0.92 1', 'yellow': '0.95 0.75 0.1 1', 'black': '0.1 0.1 0.1 1',
    'grey': '0.6 0.6 0.6 1', 'red': '0.85 0.1 0.1 1', 'teal': '0.0 0.5 0.55 1',
    'silver': '0.8 0.8 0.8 1', 'green': '0.1 0.6 0.3 1', 'purple': '0.45 0.2 0.6 1',
    'blue': '0.1 0.3 0.75 1', 'orange': '0.95 0.5 0.1 1', 'brown': '0.5 0.3 0.15 1',
}
R90 = math.pi / 2


# =====================================================================
# 부품 템플릿: 치수(mm)는 '외곽 상자' 기준, visual 좌표는 상자 바닥면 중심 기준
# =====================================================================
def vis(shape, dims, xyz, color, rpy=(0, 0, 0)):
    return dict(shape=shape, dims=dims, xyz=xyz, rpy=rpy, color=color)


def t_arduino_uno(p):
    return dict(size=[68.6, 53.4, 15], mass=25, sensor=False, visuals=[
        vis('box', [68.6, 53.4, 1.6], [0, 0, 0.8], 'teal'),
        vis('box', [16, 12, 11], [-31, 12, 7.1], 'silver'),        # USB-B (뒤쪽)
        vis('box', [13, 9, 11], [-29, -19, 7.1], 'black'),         # DC 잭
        vis('box', [35, 8, 4], [8, 0, 3.6], 'black'),              # ATmega 칩
        vis('box', [50, 2.5, 8.5], [5, 24.5, 5.85], 'black'),      # 핀헤더
        vis('box', [50, 2.5, 8.5], [5, -24.5, 5.85], 'black'),
    ])


def t_esp32_devkit_v1(p):
    """ESP32 DevKit V1 (30핀). +x 쪽 끝 = WiFi 안테나, -x 쪽 끝 = micro USB.
    standoff: 기판 아래 높이 (기본 10 = 암컷 핀헤더 소켓,
              브레드보드에 꽂으면 2.5 = 핀헤더 플라스틱 + z_offset 8.5 = 브레드보드 높이)"""
    sock = float(p.get('standoff', 10.0))
    pcb_z = sock + 0.8
    return dict(size=[51.5, 28.3, sock + 1.6 + 3.2], mass=10, sensor=False, visuals=[
        vis('box', [38.1, 2.5, sock], [-2, 11.43, sock / 2], 'black'),     # 핀헤더 소켓 (왼쪽 줄)
        vis('box', [38.1, 2.5, sock], [-2, -11.43, sock / 2], 'black'),    # 핀헤더 소켓 (오른쪽 줄)
        vis('box', [51.5, 28.3, 1.6], [0, 0, pcb_z], 'black'),             # 기판
        vis('box', [17.8, 18, 3.1], [12.0, 0, sock + 1.6 + 1.55], 'silver'),  # WROOM-32 금속 쉴드
        vis('box', [7.7, 18, 0.8], [24.8, 0, sock + 1.6 + 0.4], 'blue'),   # PCB 안테나 (끝부분)
        vis('box', [6, 8, 3], [-23.5, 0, sock + 1.6 + 1.5], 'silver'),     # micro USB
        vis('box', [4, 4, 2], [-17, 9, sock + 1.6 + 1], 'grey'),           # EN 버튼
        vis('box', [4, 4, 2], [-17, -9, sock + 1.6 + 1], 'grey'),          # BOOT 버튼
    ])


def t_breadboard_mini(p):
    """미니 브레드보드 170구 (17열 x 5+5행, 전원 레일 없음). 길이 방향 = x."""
    return dict(size=[47, 35, 8.5], mass=8, sensor=False, visuals=[
        vis('box', [47, 35, 8.5], [0, 0, 4.25], 'white'),
        vis('box', [47, 1.5, 0.3], [0, 0, 8.5], 'grey'),               # 가운데 홈
    ])


def t_breadboard_half(p):
    """하프 브레드보드 400구 (30열, 위아래 전원 레일 +/-). 길이 방향 = x."""
    return dict(size=[82, 55, 8.5], mass=30, sensor=False, visuals=[
        vis('box', [82, 55, 8.5], [0, 0, 4.25], 'white'),
        vis('box', [82, 1.5, 0.3], [0, 0, 8.5], 'grey'),               # 가운데 홈
        vis('box', [74, 0.8, 0.3], [0, 23.5, 8.5], 'red'),             # 전원 레일 +
        vis('box', [74, 0.8, 0.3], [0, 19.5, 8.5], 'blue'),            # 전원 레일 -
        vis('box', [74, 0.8, 0.3], [0, -19.5, 8.5], 'red'),
        vis('box', [74, 0.8, 0.3], [0, -23.5, 8.5], 'blue'),
    ])


def t_breadboard_full(p):
    """풀사이즈 브레드보드 830구 (63열, 위아래 전원 레일 +/-). 길이 방향 = x."""
    return dict(size=[165, 55, 8.5], mass=60, sensor=False, visuals=[
        vis('box', [165, 55, 8.5], [0, 0, 4.25], 'white'),
        vis('box', [165, 1.5, 0.3], [0, 0, 8.5], 'grey'),              # 가운데 홈
        vis('box', [155, 0.8, 0.3], [0, 23.5, 8.5], 'red'),            # 전원 레일 +
        vis('box', [155, 0.8, 0.3], [0, 19.5, 8.5], 'blue'),           # 전원 레일 -
        vis('box', [155, 0.8, 0.3], [0, -19.5, 8.5], 'red'),
        vis('box', [155, 0.8, 0.3], [0, -23.5, 8.5], 'blue'),
    ])


def t_l298n(p):
    return dict(size=[43, 43, 27], mass=30, sensor=False, visuals=[
        vis('box', [43, 43, 1.6], [0, 0, 0.8], 'red'),
        vis('box', [15, 23, 25], [8, 0, 14.1], 'black'),           # 방열판 (앞쪽)
        vis('box', [15, 8, 10], [0, 17.5, 6.6], 'blue'),           # 모터 단자
        vis('box', [15, 8, 10], [0, -17.5, 6.6], 'blue'),
        vis('box', [8, 22, 10], [-17, 0, 6.6], 'blue'),            # 전원 단자 (뒤쪽)
    ])


def t_battery_18650x2(p):
    return dict(size=[41, 77, 21], mass=110, sensor=False, visuals=[
        vis('box', [41, 77, 3], [0, 0, 1.5], 'black'),
        vis('cylinder', [9, 65], [9.5, 0, 12], 'green', (R90, 0, 0)),
        vis('cylinder', [9, 65], [-9.5, 0, 12], 'green', (R90, 0, 0)),
    ])


def t_mpu6050_gy521(p):
    """MPU-6050 6축 IMU (GY-521 모듈, 21.2 x 15.6 mm). 칩 X축 = 모듈 +x, Y축 = +y, Z축 = 위.
    핀 8개(VCC GND SCL SDA XDA XCL AD0 INT)가 -y 쪽 긴 변을 따라 아래로 나옴.
    standoff: 기판 아래 높이 (브레드보드에 꽂으면 2.5 = 핀헤더 플라스틱)"""
    sb = float(p.get('standoff', 2.5))
    top = sb + 1.6
    return dict(size=[21.2, 15.6, top + 1.0], mass=2, sensor=False, visuals=[
        vis('box', [20.3, 2.5, sb], [0, -6.3, sb / 2], 'black'),          # 핀헤더 플라스틱 (8핀)
        vis('box', [21.2, 15.6, 1.6], [0, 0, sb + 0.8], 'blue'),          # 파란 기판
        vis('box', [4, 4, 0.9], [0, 1.5, top + 0.45], 'black'),           # MPU-6050 칩
        vis('box', [3, 1.5, 1.0], [-7, 4.5, top + 0.5], 'grey'),          # 3.3V 레귤레이터
        vis('box', [6, 0.6, 0.2], [3, 6.2, top + 0.1], 'white'),          # X축 화살표 인쇄 (앞 방향 표시)
        vis('box', [1.6, 1.6, 0.2], [6.4, 6.2, top + 0.1], 'white'),
    ])


def t_ir_obstacle(p):
    """IR 장애물 감지 모듈 (FC-51 류, 31x14mm 기판 + 5mm LED 2개).
    판 아래에 기판을 붙이고 LED 가 앞(+x)을 향함. 링크 원점 = LED 앞면 (센서 frame)."""
    return dict(size=[36, 14, 8], mass=3, sensor=True, visuals=[
        vis('box', [31, 14, 1.6], [-2.5, 0, 7.2], 'blue'),                     # 기판 (판 아랫면에 붙음)
        vis('cylinder', [2.5, 8], [14, 3.5, 3.9], 'white', (0, R90, 0)),        # 적외선 LED (투명)
        vis('cylinder', [2.5, 8], [14, -3.5, 3.9], 'black', (0, R90, 0)),       # 수광 소자 (검정)
        vis('box', [6, 6, 4], [-10, 0, 4.4], 'blue'),                           # 감도 조절 가변저항
    ])


def t_tof_vl53l0x(p):
    return dict(size=[12, 25, 15], mass=5, sensor=True, visuals=[
        vis('box', [12, 25, 15], [0, 0, 7.5], 'purple'),
        vis('cylinder', [3, 2], [6, 0, 7.5], 'black', (0, R90, 0)),  # 렌즈
    ])


def t_box(p):
    L, W, H = p['size']
    return dict(size=[L, W, H], mass=p.get('mass', 10), sensor=p.get('sensor', False),
                visuals=[vis('box', [L, W, H], [0, 0, H / 2], p.get('color', 'grey'))])


def t_cylinder(p):
    d, h = p['size']   # [지름, 높이], 세워진 원기둥
    return dict(size=[d, d, h], mass=p.get('mass', 10), sensor=False,
                visuals=[vis('cylinder', [d / 2, h], [0, 0, h / 2], p.get('color', 'grey'))])


TEMPLATES = {
    'arduino_uno': t_arduino_uno, 'esp32_devkit_v1': t_esp32_devkit_v1, 'l298n': t_l298n,
    'breadboard_mini': t_breadboard_mini, 'breadboard_half': t_breadboard_half,
    'breadboard_full': t_breadboard_full, 'battery_18650x2': t_battery_18650x2,
    'mpu6050_gy521': t_mpu6050_gy521, 'ir_obstacle': t_ir_obstacle, 'tof_vl53l0x': t_tof_vl53l0x, 'box': t_box, 'cylinder': t_cylinder,
}


# =====================================================================
# 계산 도우미
# =====================================================================
def box_inertia(m, L, W, H):
    return (m / 12 * (W * W + H * H), m / 12 * (L * L + H * H), m / 12 * (L * L + W * W))


def combine(bodies):
    """bodies: [(mass, (x,y,z) 중심, (ixx,iyy,izz))] -> 합친 질량, 무게중심, 관성텐서."""
    M = sum(b[0] for b in bodies)
    c = [sum(b[0] * b[1][i] for b in bodies) / M for i in range(3)]
    I = [[0.0] * 3 for _ in range(3)]
    for m, pos, diag in bodies:
        d = [pos[i] - c[i] for i in range(3)]
        dd = sum(v * v for v in d)
        for i in range(3):
            for j in range(3):
                I[i][j] += (diag[i] if i == j else 0) + m * ((dd if i == j else 0) - d[i] * d[j])
    return M, c, I


class Box:
    """검사용 축정렬 외곽상자 (mm)."""
    def __init__(self, name, lo, hi):
        self.name, self.lo, self.hi = name, lo, hi

    def overlap(self, o):
        return [min(self.hi[i], o.hi[i]) - max(self.lo[i], o.lo[i]) for i in range(3)]


def rotated_footprint(cx, cy, L, W, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    xs, ys = [], []
    for dx, dy in ((L / 2, W / 2), (L / 2, -W / 2), (-L / 2, W / 2), (-L / 2, -W / 2)):
        xs.append(cx + c * dx - s * dy)
        ys.append(cy + s * dx + c * dy)
    return min(xs), max(xs), min(ys), max(ys)


# =====================================================================
# URDF 작성 도우미 (입력 mm, 출력 m)
# =====================================================================
def f(v):
    return f'{v:.5f}'.rstrip('0').rstrip('.') if abs(v) > 1e-12 else '0'


def xyz_m(v):
    return ' '.join(f(a / 1000) for a in v)


def rpy_s(v):
    return ' '.join(f(a) for a in v)


def geom(shape, dims):
    if shape == 'box':
        return f'<box size="{xyz_m(dims)}"/>'
    if shape == 'cylinder':
        return f'<cylinder radius="{f(dims[0] / 1000)}" length="{f(dims[1] / 1000)}"/>'
    if shape == 'sphere':
        return f'<sphere radius="{f(dims[0] / 1000)}"/>'
    raise ValueError(shape)


def visual_xml(v, shift=(0, 0, 0)):
    xyz = [v['xyz'][i] + shift[i] for i in range(3)]
    return (f'    <visual>\n      <origin xyz="{xyz_m(xyz)}" rpy="{rpy_s(v["rpy"])}"/>\n'
            f'      <geometry>{geom(v["shape"], v["dims"])}</geometry>\n'
            f'      <material name="{v["color"]}"/>\n    </visual>\n')


def collision_xml(shape, dims, xyz, rpy=(0, 0, 0)):
    return (f'    <collision>\n      <origin xyz="{xyz_m(xyz)}" rpy="{rpy_s(rpy)}"/>\n'
            f'      <geometry>{geom(shape, dims)}</geometry>\n    </collision>\n')


def inertial_xml(mass_g, com_mm, I_gmm2):
    """I 는 g·mm² 단위 3x3 -> kg·m² (1e-9)."""
    k = 1e-9
    return (f'    <inertial>\n      <origin xyz="{xyz_m(com_mm)}" rpy="0 0 0"/>\n'
            f'      <mass value="{f(mass_g / 1000)}"/>\n'
            f'      <inertia ixx="{I_gmm2[0][0] * k:.4e}" ixy="{I_gmm2[0][1] * k:.4e}" '
            f'ixz="{I_gmm2[0][2] * k:.4e}" iyy="{I_gmm2[1][1] * k:.4e}" '
            f'iyz="{I_gmm2[1][2] * k:.4e}" izz="{I_gmm2[2][2] * k:.4e}"/>\n    </inertial>\n')


def diag(d):
    return [[d[0], 0, 0], [0, d[1], 0], [0, 0, d[2]]]


def link_xml(name, body):
    return f'  <link name="{name}">\n{body}  </link>\n'


def joint_xml(name, jtype, child, xyz, rpy=(0, 0, 0), axis=None):
    ax = f'    <axis xyz="{axis}"/>\n' if axis else ''
    return (f'  <joint name="{name}" type="{jtype}">\n    <parent link="base_link"/>\n'
            f'    <child link="{child}"/>\n    <origin xyz="{xyz_m(xyz)}" rpy="{rpy_s(rpy)}"/>\n'
            f'{ax}  </joint>\n')


# =====================================================================
# 메인 빌드
# =====================================================================
def build(cfg):
    msgs = []   # (level, text)  level: OK / WARN / ERR / INFO

    d, pl, cs = cfg['drive'], cfg['plate'], cfg['caster']
    R, ww, sep = d['wheel_radius'], d['wheel_width'], d['wheel_separation']
    mo = d['motor']
    mL, mW, mH = mo['size']
    ground = -R
    plate_bot = pl['bottom_z']
    plate_top = plate_bot + pl['thickness']
    plate_cx = pl['rear_x'] + pl['length'] / 2
    plate_front = pl['rear_x'] + pl['length']

    boxes = []          # 검사용 상자
    mass_items = []     # 무게중심 계산용 (mass g, center mm)
    xml = []

    # ---------- base_link: 판 + 모터 ----------
    wheel_in = sep / 2 - ww / 2
    motor_out = wheel_in - mo['gap_to_wheel']
    motor_cy = motor_out - mW / 2
    base_vis = [vis('box', [pl['length'], pl['width'], pl['thickness']],
                    [plate_cx, 0, plate_bot + pl['thickness'] / 2], 'white')]
    base_bodies = [(pl['mass'], (plate_cx, 0, plate_bot + pl['thickness'] / 2),
                    box_inertia(pl['mass'], pl['length'], pl['width'], pl['thickness']))]
    for side, sgn in (('L', 1), ('R', -1)):
        mc = (mo['center_x'], sgn * motor_cy, mo['center_z'])
        base_vis.append(vis('box', [mL, mW, mH], list(mc), 'yellow'))
        shaft_len = mo['gap_to_wheel'] + 4
        base_vis.append(vis('cylinder', [2.7, shaft_len],
                            [0, sgn * (motor_out + shaft_len / 2), 0], 'white', (R90, 0, 0)))
        base_bodies.append((mo['mass'], mc, box_inertia(mo['mass'], mL, mW, mH)))
        boxes.append(Box(f'motor_{side}', [mc[0] - mL / 2, mc[1] - mW / 2, mc[2] - mH / 2],
                         [mc[0] + mL / 2, mc[1] + mW / 2, mc[2] + mH / 2]))
        mass_items.append((mo['mass'], mc))
    mass_items.append((pl['mass'], (plate_cx, 0, plate_bot + pl['thickness'] / 2)))
    plate_box = Box('plate', [pl['rear_x'], -pl['width'] / 2, plate_bot],
                    [plate_front, pl['width'] / 2, plate_top])
    bM, bC, bI = combine(base_bodies)
    body = ''.join(visual_xml(v) for v in base_vis)
    body += collision_xml('box', [pl['length'], pl['width'], pl['thickness']],
                          [plate_cx, 0, plate_bot + pl['thickness'] / 2])
    body += inertial_xml(bM, bC, bI)
    xml.append(link_xml('base_link', body))

    # ---------- 바퀴 ----------
    wm = d['wheel_mass']
    wI = (wm / 12 * (3 * R * R + ww * ww), wm * R * R / 2, wm / 12 * (3 * R * R + ww * ww))
    for side, sgn in (('left', 1), ('right', -1)):
        body = (visual_xml(vis('cylinder', [R, ww], [0, 0, 0], 'black', (R90, 0, 0)))
                + visual_xml(vis('cylinder', [R * 0.67, ww + 1], [0, 0, 0], 'yellow', (R90, 0, 0)))
                + visual_xml(vis('box', [R * 1.33, 1, 5], [0, sgn * (ww / 2 + 1), 0], 'white'))
                + collision_xml('cylinder', [R, ww], [0, 0, 0], (R90, 0, 0))
                + inertial_xml(wm, [0, 0, 0], diag(wI)))
        xml.append(link_xml(f'{side}_wheel', body))
        xml.append(joint_xml(f'{side}_wheel_joint', 'continuous', f'{side}_wheel',
                             [0, sgn * sep / 2, 0], axis='0 1 0'))
        boxes.append(Box(f'{side}_wheel', [-R, sgn * sep / 2 - ww / 2, -R],
                         [R, sgn * sep / 2 + ww / 2, R]))
        mass_items.append((wm, (0, sgn * sep / 2, 0)))

    # ---------- 캐스터 ----------
    br = cs['ball_radius']
    c_center = [cs['x'], cs['y'], ground + br]
    bracket_h = plate_bot - c_center[2]
    body = visual_xml(vis('sphere', [br], [0, 0, 0], 'grey'))
    if bracket_h > 0:
        body += visual_xml(vis('box', [20, 20, bracket_h], [0, 0, bracket_h / 2], 'silver'))
    else:
        msgs.append(('ERR', f'캐스터 공({br*2:.0f}mm)이 판 아래 공간보다 큼: 판 아랫면이 공 중심보다 낮음'))
    cI = 0.4 * cs['mass'] * br * br
    body += collision_xml('sphere', [br], [0, 0, 0]) + inertial_xml(cs['mass'], [0, 0, 0], diag((cI, cI, cI)))
    xml.append(link_xml('caster', body))
    xml.append(joint_xml('caster_joint', 'fixed', 'caster', c_center))
    caster_box = Box('caster', [cs['x'] - 10, cs['y'] - 10, ground], [cs['x'] + 10, cs['y'] + 10, plate_bot])
    mass_items.append((cs['mass'], c_center))

    # ---------- 부품 ----------
    part_rows = []
    part_boxes = []
    names = set()
    for p in cfg.get('parts', []) or []:
        name = p['name']
        if name in names or name in ('base_link', 'caster', 'left_wheel', 'right_wheel'):
            msgs.append(('ERR', f'부품 이름 중복: {name}'))
            continue
        names.add(name)
        if p['type'] not in TEMPLATES:
            msgs.append(('ERR', f'{name}: 알 수 없는 type "{p["type"]}" (가능: {", ".join(TEMPLATES)})'))
            continue
        t = TEMPLATES[p['type']](p)
        if 'size' in p and p['type'] not in ('box', 'cylinder'):
            t['size'] = p['size']
        mass = p.get('mass', t['mass'])
        L, W, H = t['size']
        mount, off = p.get('mount', 'top'), p.get('z_offset', 0)
        if mount == 'top':
            zb = plate_top + off
        elif mount == 'bottom':
            zb = plate_bot - off - H
        elif mount == 'free':
            zb = p['z']
        else:
            msgs.append(('ERR', f'{name}: mount 는 top/bottom/free 중 하나'))
            continue
        yaw = math.radians(p.get('yaw', 0))
        cx, cy = p['x'], p['y']
        if t['sensor']:   # 링크 원점 = 센서 앞면 중앙
            origin = [cx + math.cos(yaw) * L / 2, cy + math.sin(yaw) * L / 2, zb + H / 2]
            shift = (-L / 2, 0, -H / 2)
        else:             # 링크 원점 = 부품 바닥면 중앙
            origin = [cx, cy, zb]
            shift = (0, 0, 0)
        body = ''.join(visual_xml(v, shift) for v in t['visuals'])
        ctr = [shift[0], shift[1], shift[2] + H / 2]
        body += collision_xml('box', [L, W, H], ctr)
        body += inertial_xml(mass, ctr, diag(box_inertia(mass, L, W, H)))
        xml.append(link_xml(name, body))
        xml.append(joint_xml(f'{name}_joint', 'fixed', name, origin, (0, 0, yaw)))

        x0, x1, y0, y1 = rotated_footprint(cx, cy, L, W, yaw)
        b = Box(name, [x0, y0, zb], [x1, y1, zb + H])
        part_boxes.append((b, p, t))
        mass_items.append((mass, (cx, cy, zb + H / 2)))
        part_rows.append((name, p['type'], mount, cx, cy, zb, zb + H, mass, t['sensor']))

    # =================================================================
    # 검사
    # =================================================================
    TOL = 0.5
    fixed = [b for b in boxes] + [caster_box]
    pb = [b for b, _, _ in part_boxes]
    clash = False
    for i, a in enumerate(pb):
        for o in pb[i + 1:] + fixed + [plate_box]:
            ov = a.overlap(o)
            if all(v > TOL for v in ov):
                clash = True
                msgs.append(('ERR', f'겹침: {a.name} <-> {o.name}  '
                                    f'(x {ov[0]:.1f} / y {ov[1]:.1f} / z {ov[2]:.1f} mm 겹침)'))
    if not clash:
        msgs.append(('OK', '부품끼리 / 바퀴·모터·캐스터·판과 겹침 없음'))

    for b, p, t in part_boxes:
        clr = b.lo[2] - ground
        if clr < 0:
            msgs.append(('ERR', f'{b.name}: 바닥 아래로 {-clr:.1f} mm 파고듦'))
        elif clr < 10:
            msgs.append(('WARN', f'{b.name}: 바닥과 {clr:.1f} mm (문턱에 걸릴 수 있음)'))
        if p.get('mount') in ('top', 'bottom'):
            over = max(pl['rear_x'] - b.lo[0], b.hi[0] - plate_front,
                       -pl['width'] / 2 - b.lo[1], b.hi[1] - pl['width'] / 2)
            if over > TOL:
                lvl = 'INFO' if t['sensor'] else 'WARN'
                msgs.append((lvl, f'{b.name}: 판 밖으로 {over:.1f} mm 튀어나옴'
                                  + (' (센서 앞면 노출용이면 정상)' if t['sensor'] else '')))
    mclr = min(b.lo[2] for b in boxes if b.name.startswith('motor')) - ground
    msgs.append(('OK' if mclr >= 10 else 'WARN', f'모터 바닥과 지면 사이 {mclr:.1f} mm'))
    if pl['width'] / 2 > wheel_in and pl['rear_x'] < R and plate_front > -R and plate_bot < R:
        msgs.append(('WARN', f'판 폭({pl["width"]}mm)이 바퀴 안쪽면 간격({wheel_in*2:.0f}mm)보다 넓음 '
                             '-> 판에 바퀴 홈이 없으면 바퀴와 닿음'))
    if not (pl['rear_x'] <= cs['x'] <= plate_front):
        msgs.append(('WARN', f'캐스터 x={cs["x"]} 가 판 범위({pl["rear_x"]}~{plate_front}) 밖'))

    # ---------- 무게중심 / 안정성 ----------
    M = sum(m for m, _ in mass_items)
    com = [sum(m * c[i] for m, c in mass_items) / M for i in range(3)]
    tri = [(0, sep / 2), (0, -sep / 2), (cs['x'], cs['y'])]

    def edge_dist(p, a, b):  # 삼각형 안쪽이면 +
        (x0, y0), (x1, y1) = a, b
        cross = (x1 - x0) * (p[1] - y0) - (y1 - y0) * (p[0] - x0)
        return cross / math.hypot(x1 - x0, y1 - y0)
    ds = [edge_dist(com, tri[i], tri[(i + 1) % 3]) for i in range(3)]
    if ds[0] < 0:   # 꼭짓점 순서가 시계방향이면 부호 반전
        ds = [-v for v in ds]
    margin = min(ds)
    h = com[2] - ground
    stats = dict(M=M, com=com, h=h, margin=margin)
    if margin <= 0:
        msgs.append(('ERR', f'무게중심이 지지 삼각형(바퀴 2 + 캐스터) 밖: 그대로 넘어짐'))
    else:
        lvl = 'OK' if margin >= 15 else 'WARN'
        msgs.append((lvl, f'무게중심이 지지 삼각형 안쪽, 가장 가까운 변까지 {margin:.1f} mm'))
        if cs['x'] > 0:
            a_acc, a_brk = com[0] / h, (cs['x'] - com[0]) / h
            lab_acc, lab_brk = '급출발 시 캐스터(앞) 들림', '급정지 시 뒷바퀴 들림'
        else:
            a_acc, a_brk = (com[0] - cs['x']) / h, -com[0] / h
            lab_acc, lab_brk = '급출발 시 뒷바퀴 들림', '급정지 시 캐스터(뒤) 들림'
        for lab, a in ((lab_acc, a_acc), (lab_brk, a_brk)):
            msgs.append(('OK' if a >= 0.5 else 'WARN', f'{lab} 한계 가속도 {a:.2f} g'
                         + ('' if a >= 0.5 else ' (낮음: 이 정도 가속/감속이면 들릴 수 있음)')))
        stats['caster_load'] = (com[0] / cs['x']) if cs['x'] else 0

    urdf = ('<?xml version="1.0"?>\n'
            '<!-- AUTO-GENERATED by tools/build_robot.py from config/robot_layout.yaml\n'
            '     DO NOT EDIT: edit robot_layout.yaml and re-run the tool. -->\n'
            '<robot name="robot_car">\n'
            + ''.join(f'  <material name="{k}"><color rgba="{v}"/></material>\n' for k, v in COLORS.items())
            + '\n' + '\n'.join(xml) + '</robot>\n')
    params = ('# AUTO-GENERATED by tools/build_robot.py from config/robot_layout.yaml - DO NOT EDIT\n'
              f'WHEEL_RADIUS = {R / 1000!r}        # [m]\n'
              f'WHEEL_SEPARATION = {sep / 1000!r}    # [m]\n'
              f'MAX_WHEEL_SPEED = {d["max_rpm"] * 2 * math.pi / 60:.4f}  # [rad/s] ({d["max_rpm"]} rpm)\n')
    return urdf, params, msgs, part_rows, stats


# =====================================================================
# PyBullet 주행 테스트
# =====================================================================
def sim_test(urdf_path, params):
    try:
        import pybullet as p
        import pybullet_data
    except ImportError:
        return None, '(pybullet 없음: source env.sh 후 실행하면 주행 테스트도 합니다)'
    ns = {}
    exec(params, ns)
    r_, sep, vmax = ns['WHEEL_RADIUS'], ns['WHEEL_SEPARATION'], ns['MAX_WHEEL_SPEED']

    def wheels(v, w):
        wl, wr = (v - w * sep / 2) / r_, (v + w * sep / 2) / r_
        k = max(1, abs(wl) / vmax, abs(wr) / vmax)
        return wl / k, wr / k

    cid = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1 / 240)
    p.loadURDF('plane.urdf')
    rob = p.loadURDF(urdf_path, [0, 0, r_ + 0.005], flags=p.URDF_USE_INERTIA_FROM_FILE)
    J = {p.getJointInfo(rob, i)[1].decode(): i for i in range(p.getNumJoints(rob))}
    for w in ('left_wheel_joint', 'right_wheel_joint'):
        p.changeDynamics(rob, J[w], lateralFriction=1.0)
    p.changeDynamics(rob, J['caster_joint'], lateralFriction=0, spinningFriction=0, rollingFriction=0)
    # PyBullet 은 base 자세를 '관성 주축' 기준으로 돌려주므로 base_link 기준으로 변환
    info = p.getDynamicsInfo(rob, -1)
    com_to_link = p.invertTransform(info[3], info[4])

    def link_pose():
        pos, orn = p.getBasePositionAndOrientation(rob)
        return p.multiplyTransforms(pos, orn, *com_to_link)
    rows = []
    for label, v, w, sec in (('안착', 0, 0, 1), ('전진 0.3m/s 2s', 0.3, 0, 2), ('급정지', 0, 0, 0.7),
                             ('후진 0.3m/s 2s', -0.3, 0, 2), ('급정지', 0, 0, 0.7),
                             ('최고속 전진 2s', 1.0, 0, 2), ('급정지', 0, 0, 1),
                             ('제자리 좌회전 1s', 0, 2.0, 1), ('제자리 우회전 1s', 0, -2.0, 1)):
        wl, wr = wheels(v, w)
        tilt = 0
        x0, o0 = link_pose()
        yaw0 = p.getEulerFromQuaternion(o0)[2]
        for _ in range(int(sec * 240)):
            p.setJointMotorControl2(rob, J['left_wheel_joint'], p.VELOCITY_CONTROL, targetVelocity=wl, force=0.3)
            p.setJointMotorControl2(rob, J['right_wheel_joint'], p.VELOCITY_CONTROL, targetVelocity=wr, force=0.3)
            p.stepSimulation()
            e = p.getEulerFromQuaternion(link_pose()[1])
            tilt = max(tilt, abs(math.degrees(e[0])), abs(math.degrees(e[1])))
        pos, orn = link_pose()
        dist = math.hypot(pos[0] - x0[0], pos[1] - x0[1])
        dyaw = math.degrees((p.getEulerFromQuaternion(orn)[2] - yaw0 + math.pi) % (2 * math.pi) - math.pi)
        # 명령대로 움직였는지: 이론값 (최고속 제한 반영) 대비 실제
        v_eff = (wl + wr) / 2 * r_
        w_eff = math.degrees((wr - wl) * r_ / sep)
        if abs(v_eff) > 0:
            expect, actual, unit = abs(v_eff) * sec, dist, 'cm'
        elif abs(w_eff) > 0:
            expect, actual, unit = abs(w_eff) * sec, abs(dyaw), '°'
        else:
            expect = actual = unit = None
        rows.append((label, dist, dyaw, tilt, expect, actual, unit))
    p.disconnect(cid)
    return rows, ''


# =====================================================================
# 실행 중인 RViz 갱신 (robot_state_publisher 파라미터 교체)
# =====================================================================
def reload_rviz(urdf):
    try:
        import rclpy
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters
    except ImportError:
        return 'rclpy 없음 (source env.sh 필요)'
    rclpy.init()
    node = rclpy.create_node('build_robot_reloader')
    try:
        cli = node.create_client(SetParameters, '/robot_state_publisher/set_parameters')
        if not cli.wait_for_service(timeout_sec=1.5):
            return 'RViz(robot_state_publisher) 꺼져 있음 -> ros2 launch robot_car display.launch.py mode:=model'
        req = SetParameters.Request()
        req.parameters = [Parameter(name='robot_description', value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=urdf))]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
        res = fut.result()
        if res and res.results[0].successful:
            return '실행 중인 RViz 에 새 모델 반영 완료'
        return f'갱신 실패: {res.results[0].reason if res else "응답 없음"}'
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


# =====================================================================
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--layout', default=LAYOUT)
    ap.add_argument('--no-sim', action='store_true')
    ap.add_argument('--no-reload', action='store_true')
    a = ap.parse_args()

    with open(a.layout) as fp:
        cfg = yaml.safe_load(fp)
    urdf, params, msgs, rows, st = build(cfg)
    with open(URDF_OUT, 'w') as fp:
        fp.write(urdf)
    with open(PARAMS_OUT, 'w') as fp:
        fp.write(params)

    R = cfg['drive']['wheel_radius']
    print(f'\n== 부품 배치 (mm, 원점=뒷바퀴 축 중앙, 바닥 z={-R}) ' + '=' * 20)
    print(f'{"이름":<17}{"종류":<17}{"장착":<7}{"x":>7}{"y":>7}{"z범위":>15}{"무게g":>7}')
    for n, t, m, x, y, z0, z1, mass, sensor in rows:
        print(f'{n:<17}{t:<17}{m:<7}{x:>7.1f}{y:>7.1f}{f"{z0:.1f}~{z1:.1f}":>15}{mass:>7.0f}'
              + ('  [센서]' if sensor else ''))

    print('\n== 검사 ' + '=' * 50)
    tag = {'OK': '[ OK ]', 'WARN': '[주의]', 'ERR': '[오류]', 'INFO': '[참고]'}
    for lvl, text in msgs:
        print(f'{tag[lvl]} {text}')
    c = st['com']
    print(f'\n총 무게 {st["M"]:.0f} g | 무게중심 x={c[0]:.1f} y={c[1]:.1f} mm, 바닥에서 높이 {st["h"]:.1f} mm'
          + (f' | 캐스터 하중 {st["caster_load"]*100:.0f}%' if 'caster_load' in st else ''))

    if not a.no_sim:
        print('\n== PyBullet 주행 테스트 ' + '=' * 34)
        res, note = sim_test(URDF_OUT, params)
        if res is None:
            print(note)
        else:
            problems = []
            for label, dist, yaw, tilt, expect, actual, unit in res:
                flag = ''
                if tilt >= 10:
                    flag += '  <- 넘어짐/들림!'
                    problems.append(f'{label}: 최대 {tilt:.0f}° 기울어짐')
                if expect:
                    ratio = actual / expect
                    flag += f'  (명령 대비 {ratio*100:3.0f}%)'
                    if ratio < 0.7:
                        flag += ' <- 바퀴 헛돎/걸림!'
                        problems.append(f'{label}: 명령의 {ratio*100:.0f}%만 움직임 '
                                        '(뒷바퀴 하중 부족이나 바닥에 걸리는 부품 확인)')
                    if unit == 'cm' and abs(yaw) > 5:
                        flag += ' <- 직진인데 휨!'
                        problems.append(f'{label}: 직진 중 {yaw:.0f}° 틀어짐')
                print(f'{label:<16} 이동 {dist*100:6.1f} cm  방향변화 {yaw:7.1f}°  최대기울기 {tilt:5.1f}°{flag}')
            if problems:
                for t in problems:
                    msgs.append(('ERR', '주행 ' + t))
                    print('[오류] 주행 ' + t)
            else:
                print('[ OK ] 명령대로 주행, 넘어짐/헛돎 없음')

    print(f'\n생성: {os.path.relpath(URDF_OUT, ROOT)}, {os.path.relpath(PARAMS_OUT, ROOT)}')
    if not a.no_reload:
        print('RViz: ' + reload_rviz(urdf))
    n_err = sum(1 for lv, _ in msgs if lv == 'ERR')
    sys.exit(1 if n_err else 0)


if __name__ == '__main__':
    main()
