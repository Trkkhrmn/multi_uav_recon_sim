#!/usr/bin/env python3
# MRTA panel - ustte harita altta drone kartlari
# /mrta/status topicinden json alip ciziyor
# pygame ile yazdim cunku rqt cok karisik geldi

import json
import math
import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String

try:
    import pygame
except ImportError:
    print("PyGame not installed. Install: pip install pygame")
    sys.exit(1)

BG           = (30,  30,  35)
PANEL_BG     = (42,  42,  50)
HEADER_BG    = (22,  22,  28)
MAP_BG       = (18,  18,  22)
GRID_COLOR   = (40,  40,  48)
WHITE        = (240, 240, 240)
GRAY         = (140, 140, 140)
DARK_GRAY    = (80,  80,  80)
GREEN        = (50,  205, 50)
YELLOW       = (255, 200, 0)
RED          = (220, 50,  50)
BLUE_C       = (80,  140, 255)
MAGENTA_C    = (220, 50,  220)
CYAN_C       = (0,   210, 210)
CYAN         = (0,   200, 200)
ORANGE       = (255, 160, 40)
BASE_COLOR   = (255, 220, 80)
COMPLETED_C  = (60,  60,  60)

STATE_COLORS = {
    'IDLE':             GRAY,
    'ARMING':           YELLOW,
    'CLIMB':            CYAN,
    'GO':               GREEN,
    'DESCEND':          ORANGE,
    'DROP':             RED,
    'AFTER_DROP':       ORANGE,
    'CLIMB_AFTER_DROP': CYAN,
    'RETURN_TO_BASE':   YELLOW,
    'RESUPPLY_WAIT':    MAGENTA_C,
    'RTH_LANDED':       DARK_GRAY,
}

TARGET_TYPE_COLORS = {0: BLUE_C, 1: MAGENTA_C, 2: CYAN_C}
TARGET_TYPE_LABELS = {0: 'B', 1: 'M', 2: 'C'}

DRONE_COLORS = {
    5:  (255, 100, 100),
    6:  (255, 180, 80),
    7:  (100, 220, 100),
    8:  (80,  180, 255),
    9:  (200, 120, 255),
    10: (255, 255, 120),
}


# hiz etiketi goster (yavas/orta/hizli)
def speed_label(s):
    if s <= 0.55:
        return 'SLOW'
    if s <= 0.75:
        return 'MED'
    return 'FAST'

def speed_color(s):
    if s <= 0.55:
        return RED
    if s <= 0.75:
        return YELLOW
    return GREEN


def draw_dashed_line(surface, color, start, end, dash_len=8, gap_len=6, phase=0):
    x1, y1 = start
    x2, y2 = end
    dx, dy = x2 - x1, y2 - y1
    dist = math.hypot(dx, dy)
    if dist < 1e-3:
        return
    ux, uy = dx / dist, dy / dist
    pos = phase % (dash_len + gap_len)
    d = -pos
    while d < dist:
        seg_start = max(d, 0)
        seg_end = min(d + dash_len, dist)
        if seg_end > 0:
            sx = x1 + ux * seg_start
            sy = y1 + uy * seg_start
            ex = x1 + ux * seg_end
            ey = y1 + uy * seg_end
            pygame.draw.line(surface, color, (sx, sy), (ex, ey), 2)
        d += dash_len + gap_len


def draw_dashed_circle(surface, center, radius, color, width=2, dash_deg=22, gap_deg=14):
    cx, cy = center
    rect = pygame.Rect(cx - radius, cy - radius, radius * 2, radius * 2)
    angle = 0
    while angle < 360:
        start = math.radians(angle)
        end = math.radians(min(angle + dash_deg, 360))
        pygame.draw.arc(surface, color, rect, start, end, width)
        angle += dash_deg + gap_deg


def draw_glow(surface, center, radius, color, alpha=90):
    glow = pygame.Surface((radius * 4, radius * 4), pygame.SRCALPHA)
    pygame.draw.circle(glow, (*color, alpha), (radius * 2, radius * 2), radius * 2)
    surface.blit(glow, (center[0] - radius * 2, center[1] - radius * 2))


class MRTAPanelNode(Node):
    def __init__(self):
        super().__init__('mrta_panel')
        self.declare_parameter('status_topic', '/mrta/status')
        topic = self.get_parameter('status_topic').value
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(String, topic, self._cb, qos)
        self.latest_data = None
        self.lock = threading.Lock()

    def _cb(self, msg):
        try:
            with self.lock:
                self.latest_data = json.loads(msg.data)
        except Exception:
            pass

    def get_data(self):
        with self.lock:
            return self.latest_data


# harita sinirlari (world koordinat)
MAP_MIN_X, MAP_MAX_X = -80.0, 85.0
MAP_MIN_Y, MAP_MAX_Y = -70.0, 75.0
MAP_PAD = 12


# world (x,y) -> ekrandaki pixel (x,y) cevirmek icin
def w2m(wx, wy, mr):
    mx, my, mw, mh = mr
    px = mx + MAP_PAD + (wx - MAP_MIN_X) / (MAP_MAX_X - MAP_MIN_X) * (mw - 2 * MAP_PAD)
    py = my + MAP_PAD + (MAP_MAX_Y - wy) / (MAP_MAX_Y - MAP_MIN_Y) * (mh - 2 * MAP_PAD)
    return int(px), int(py)


# ana dongu - pygame ile cizim
def run_panel():
    rclpy.init()
    node = MRTAPanelNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    pygame.init()
    W, H = 1280, 900
    screen = pygame.display.set_mode((W, H), pygame.RESIZABLE)
    pygame.display.set_caption('MRTA Command Panel')
    clock = pygame.time.Clock()

    fb = pygame.font.SysFont('DejaVu Sans', 20, bold=True)
    fm = pygame.font.SysFont('DejaVu Sans', 14, bold=True)
    fs = pygame.font.SysFont('DejaVu Sans', 12)
    ft = pygame.font.SysFont('DejaVu Sans', 10)

    trails = {}
    MAX_TRAIL = 300

    running = True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.VIDEORESIZE:
                W, H = e.w, e.h
                screen = pygame.display.set_mode((W, H), pygame.RESIZABLE)

        screen.fill(BG)
        data = node.get_data()

        # ust baslik
        pygame.draw.rect(screen, HEADER_BG, (0, 0, W, 38))
        screen.blit(fb.render('MRTA Command Panel', True, WHITE), (12, 8))

        if data is None:
            screen.blit(fm.render('Waiting for data... (/mrta/status)', True, GRAY),
                        (W // 2 - 140, H // 2))
            pygame.display.flip()
            clock.tick(10)
            continue

        workers = data.get('workers', [])
        targets = data.get('targets', [])
        t_total = data.get('targets_total', 0)
        t_comp  = data.get('targets_completed', 0)
        t_pend  = data.get('targets_pending', 0)
        base_x  = data.get('base_x', 188.0)
        base_y  = data.get('base_y', 54.5)

        # Target summary (in header)
        summ = f'Targets: {t_total} total | {t_comp} completed | {t_pend} pending'
        screen.blit(fs.render(summ, True, CYAN), (W - fs.size(summ)[0] - 12, 12))

        # layout: ustte harita altta kartlar
        CARD_AREA_H = 240
        map_h = H - 38 - CARD_AREA_H - 8
        if map_h < 200:
            map_h = 200
        mr = (6, 42, W - 12, map_h)
        card_y0 = 42 + map_h + 4

        # harita alani
        mx, my, mw, mh = mr
        pygame.draw.rect(screen, MAP_BG, mr, border_radius=6)

        # Grid
        for xw in range(int(MAP_MIN_X), int(MAP_MAX_X) + 1, 20):
            pygame.draw.line(screen, GRID_COLOR, w2m(xw, MAP_MIN_Y, mr), w2m(xw, MAP_MAX_Y, mr), 1)
        for yw in range(int(MAP_MIN_Y), int(MAP_MAX_Y) + 1, 20):
            pygame.draw.line(screen, GRID_COLOR, w2m(MAP_MIN_X, yw, mr), w2m(MAP_MAX_X, yw, mr), 1)

        # Axis labels
        for xw in range(int(MAP_MIN_X), int(MAP_MAX_X) + 1, 40):
            p = w2m(xw, MAP_MIN_Y, mr)
            l = ft.render(str(xw), True, DARK_GRAY)
            screen.blit(l, (p[0] - l.get_width() // 2, my + mh - 12))
        for yw in range(int(MAP_MIN_Y), int(MAP_MAX_Y) + 1, 40):
            p = w2m(MAP_MIN_X, yw, mr)
            screen.blit(ft.render(str(yw), True, DARK_GRAY), (mx + 2, p[1] - 5))

        # Base
        bp = w2m(base_x, base_y, mr)
        pygame.draw.rect(screen, BASE_COLOR, (bp[0] - 7, bp[1] - 7, 14, 14), border_radius=3)
        pygame.draw.rect(screen, WHITE, (bp[0] - 7, bp[1] - 7, 14, 14), 2, border_radius=3)
        screen.blit(ft.render('BASE', True, BASE_COLOR), (bp[0] - 8, bp[1] + 9))

        # Targets (priority view)
        now_ms = pygame.time.get_ticks()
        for t in targets:
            tx, ty, tt = t['x'], t['y'], t.get('type', 0)
            comp = t.get('completed', False)
            drops, req = t.get('drops', 0), t.get('required', 3)
            priority = t.get('priority', 'NORMAL')
            tp = w2m(tx, ty, mr)
            col = TARGET_TYPE_COLORS.get(tt, WHITE)

            if comp:
                pygame.draw.circle(screen, COMPLETED_C, tp, 9)
                pygame.draw.line(screen, GREEN, (tp[0] - 4, tp[1]), (tp[0] - 1, tp[1] + 3), 2)
                pygame.draw.line(screen, GREEN, (tp[0] - 1, tp[1] + 3), (tp[0] + 5, tp[1] - 4), 2)
                continue

            # PRIORITY: thick edge + light glow
            if priority == 'PRIORITY':
                pulse = 2 + int(2 * (1 + math.sin(now_ms / 400.0)) / 2)
                draw_glow(screen, tp, 10 + pulse, col, alpha=70)
                pygame.draw.circle(screen, col, tp, 12 + pulse, 0)
                pygame.draw.circle(screen, WHITE, tp, 12 + pulse, 3)
            # DECOY: dim + dashed edge
            elif priority == 'DECOY':
                pygame.draw.circle(screen, col, tp, 9, 0)
                draw_dashed_circle(screen, tp, 12, (120, 120, 120), width=2)
            # NORMAL: thin edge
            else:
                pygame.draw.circle(screen, col, tp, 10, 0)
                pygame.draw.circle(screen, WHITE, tp, 10, 1)

            lbl = ft.render(TARGET_TYPE_LABELS.get(tt, '?'), True, WHITE)
            screen.blit(lbl, (tp[0] - lbl.get_width() // 2, tp[1] - 5))
            pr = ft.render(f'{drops}/{req}', True, col)
            screen.blit(pr, (tp[0] - pr.get_width() // 2, tp[1] + 12))

        # Drones + trail + assignment line
        progress_states = ('GO', 'DESCEND', 'DROP', 'AFTER_DROP', 'CLIMB_AFTER_DROP')
        assigned_states = ('ARMING', 'CLIMB')
        for w in workers:
            vid = w.get('id', 0)
            px, py_ = w.get('pos_x', 0), w.get('pos_y', 0)
            tx, ty = w.get('target_x'), w.get('target_y')
            state = w.get('state', 'IDLE')
            dc = DRONE_COLORS.get(vid, WHITE)
            dp = w2m(px, py_, mr)

            # Trail
            trails.setdefault(vid, []).append(dp)
            if len(trails[vid]) > MAX_TRAIL:
                trails[vid] = trails[vid][-MAX_TRAIL:]
            tr = trails[vid]
            if len(tr) > 2:
                for j in range(1, len(tr)):
                    a = int(50 * j / len(tr))
                    c = tuple(min(255, dc[k] // 5 + a) for k in range(3))
                    pygame.draw.line(screen, c, tr[j - 1], tr[j], 1)

            # Assignment line (style by state)
            if tx is not None and ty is not None and state not in ('IDLE', 'RTH_LANDED'):
                tp = w2m(tx, ty, mr)
                if state in progress_states:
                    phase = (now_ms // 30) % 100
                    draw_dashed_line(screen, dc, dp, tp, dash_len=8, gap_len=6, phase=phase)
                elif state in assigned_states:
                    pygame.draw.line(screen, dc, dp, tp, 2)
                else:
                    pygame.draw.line(screen, dc, dp, tp, 2)

            # Drone icon
            sz = 8
            if state in ('GO', 'DESCEND', 'DROP', 'AFTER_DROP', 'CLIMB_AFTER_DROP'):
                if tx is not None and ty is not None:
                    tp2 = w2m(tx, ty, mr)
                    ang = math.atan2(tp2[1] - dp[1], tp2[0] - dp[0])
                else:
                    ang = 0
                pts = [(dp[0] + int(sz * math.cos(ang + o)), dp[1] + int(sz * math.sin(ang + o)))
                       for o in (0, 2.4, -2.4)]
                pygame.draw.polygon(screen, dc, pts)
            else:
                pygame.draw.circle(screen, dc, dp, 6)
            pygame.draw.circle(screen, WHITE, dp, sz + 1, 1)
            screen.blit(ft.render(f'W{vid}', True, dc), (dp[0] + 10, dp[1] - 5))

        # Map title + legend
        screen.blit(fm.render('Live Map', True, WHITE), (mx + 10, my + 3))
        lx = mx + mw - 155
        ly = my + mh - 72
        pygame.draw.rect(screen, (30, 30, 38), (lx - 4, ly - 3, 150, 68), border_radius=4)
        for i, (tt, lb) in enumerate([(0, 'Blue (2p)'), (1, 'Magenta (1p)'), (2, 'Cyan (3p)')]):
            yy = ly + i * 16
            pygame.draw.circle(screen, TARGET_TYPE_COLORS[tt], (lx + 7, yy + 6), 5)
            screen.blit(ft.render(lb, True, TARGET_TYPE_COLORS[tt]), (lx + 18, yy))
        pygame.draw.rect(screen, BASE_COLOR, (lx + 2, ly + 50, 10, 10), border_radius=2)
        screen.blit(ft.render('Base', True, BASE_COLOR), (lx + 18, ly + 50))

        # drone kartlari 3x2 grid
        cols, rows = 3, 2
        gap = 6
        cw = (W - gap * (cols + 1)) // cols
        ch = (CARD_AREA_H - gap * (rows + 1)) // rows

        for i, w in enumerate(workers):
            col_i = i % cols
            row_i = i // cols
            cx = gap + col_i * (cw + gap)
            cy = card_y0 + gap + row_i * (ch + gap)

            vid = w.get('id', '?')
            state = w.get('state', 'IDLE')
            pkg_str = w.get('packages', '?/?')
            pkg_rem = w.get('pkg_remaining', 0)
            pkg_max = w.get('pkg_max', 1)
            spd = w.get('speed_scale', 1.0)
            px_ = w.get('pos_x', 0)
            py2 = w.get('pos_y', 0)
            pz  = w.get('pos_z', 0)
            tx_  = w.get('target_x')
            ty_  = w.get('target_y')
            tt   = w.get('target_type')
            ttl  = w.get('target_type_label', '')
            dc   = DRONE_COLORS.get(vid, WHITE)

            # Card background
            pygame.draw.rect(screen, PANEL_BG, (cx, cy, cw, ch), border_radius=6)
            pygame.draw.rect(screen, dc, (cx, cy, 4, ch), border_radius=2)

            x0 = cx + 12

            # Row 1: ID + speed + status
            screen.blit(fm.render(f'W{vid}', True, dc), (x0, cy + 4))
            screen.blit(ft.render(speed_label(spd), True, speed_color(spd)), (x0 + 32, cy + 6))
            sc = STATE_COLORS.get(state, WHITE)
            screen.blit(fm.render(state, True, sc), (x0 + 85, cy + 4))

            # Row 2: Package bar
            bx, by, bw, bh = x0, cy + 24, 70, 11
            pygame.draw.rect(screen, DARK_GRAY, (bx, by, bw, bh), border_radius=2)
            if pkg_max > 0 and pkg_rem > 0:
                fw = int(bw * pkg_rem / pkg_max)
                pygame.draw.rect(screen, GREEN, (bx, by, fw, bh), border_radius=2)
            screen.blit(ft.render(f'Load: {pkg_str}', True, WHITE), (bx + bw + 5, by - 1))
            screen.blit(ft.render(f'Cap:{pkg_max}', True, GRAY), (bx + bw + 62, by - 1))

            # Row 3: Target
            if tx_ is not None and ty_ is not None:
                tc = TARGET_TYPE_COLORS.get(tt, WHITE)
                screen.blit(fs.render(f'Target: ({tx_:.0f},{ty_:.0f}) {ttl}', True, tc), (x0, cy + 40))
                rmap = {0: 2, 1: 1, 2: 3}
                screen.blit(ft.render(f'{rmap.get(tt, "?")}p required', True, GRAY), (x0 + 200, cy + 42))
            else:
                screen.blit(fs.render('No target', True, DARK_GRAY), (x0, cy + 40))

            # Row 4: Position
            screen.blit(ft.render(f'Pos: ({px_:.0f}, {py2:.0f}, z:{pz:.0f})', True, GRAY), (x0, cy + 58))

            # Icon
            ix = cx + cw - 22
            iy = cy + ch // 2
            if state == 'GO':
                pygame.draw.polygon(screen, GREEN, [
                    (ix, iy - 7), (ix + 12, iy), (ix, iy + 7)])
            elif state in ('DROP', 'DESCEND', 'AFTER_DROP'):
                pygame.draw.circle(screen, RED, (ix + 6, iy), 7)
            elif state == 'RESUPPLY_WAIT':
                pygame.draw.rect(screen, MAGENTA_C, (ix, iy - 6, 12, 12))
            elif state in ('RETURN_TO_BASE', 'CLIMB_AFTER_DROP', 'CLIMB'):
                pygame.draw.polygon(screen, YELLOW, [
                    (ix + 6, iy - 9), (ix + 12, iy + 3), (ix, iy + 3)])
            elif state in ('IDLE', 'RTH_LANDED'):
                pygame.draw.rect(screen, DARK_GRAY, (ix, iy - 6, 12, 12), border_radius=2)
            elif state == 'ARMING':
                pygame.draw.circle(screen, YELLOW, (ix + 6, iy), 7, 2)

        pygame.display.flip()
        clock.tick(10)

    pygame.quit()
    node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    run_panel()


if __name__ == '__main__':
    main()
