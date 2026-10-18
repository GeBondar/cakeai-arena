"""Play mode: YOU drive the opponent with the keyboard, CakeAI fights back on its own.

    python -m arena play --robot flipper

W/S or ↑/↓ throttle · A/D or ←/→ steer · SPACE fire flipper / toggle spinner ·
R restart · ESC quit. Needs `pip install pygame`.
"""
import math
import time

import numpy as np

from .match import Match
from .pilots import ROSTER
from .video import Broadcast

HELP = "W/S throttle · A/D steer · SPACE weapon · R restart · ESC quit"


def play(robot="flipper", secs=120.0, fps=30, scale=None):
    try:
        import pygame
    except ImportError as e:
        raise SystemExit("Play mode needs pygame:  pip install pygame") from e
    import cv2
    pygame.init()
    pygame.display.set_caption("CakeAI Arena — you vs CakeAI")
    info = pygame.display.Info()

    def new_match():
        mt = Match(robot, secs=secs, fps=fps, seed=int(time.time()) % 1000)
        st = dict(spin=True, fire_until=-1.0)

        def human(m):
            k = pygame.key.get_pressed()
            thr = (k[pygame.K_w] or k[pygame.K_UP]) - (k[pygame.K_s] or k[pygame.K_DOWN])
            steer = (k[pygame.K_a] or k[pygame.K_LEFT]) - (k[pygame.K_d] or k[pygame.K_RIGHT])
            l, r = 0.85 * thr - 0.6 * steer, 0.85 * thr + 0.6 * steer
            n = max(1.0, abs(l), abs(r))
            if m.wkind["r1"] == "flipper":
                w = m.flip_angle if m.t < st["fire_until"] else 0.0
            else:
                w = 1.0 if st["spin"] else 0.0
            return l / n, r / n, w
        mt.human = human
        return mt, st

    mt, st = new_match()
    bc = Broadcast(mt, h=540, w_ai=540, w_tv=720)
    s = scale or min(1.0, (info.current_w - 40) / bc.W, (info.current_h - 120) / (bc.H + 30))
    size = (int(bc.W * s), int((bc.H + 30) * s))
    screen = pygame.display.set_mode(size)
    small = pygame.font.SysFont(None, max(16, int(24 * s)))
    clock_wall = time.time()
    running = True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                running = False
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_SPACE:
                if mt.wkind["r1"] == "flipper":
                    st["fire_until"] = mt.t + 0.25
                else:
                    st["spin"] = not st["spin"]
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_r:
                bc.close(); mt.close()
                mt, st = new_match()
                bc = Broadcast(mt, h=540, w_ai=540, w_tv=720)
                clock_wall = time.time()
        done = mt.step()
        frame = bc.frame()
        if done:
            res = mt.finish()
            frame = bc.card([f"WINNER: {res['winner']}", res["reason"], "press R to fight again, ESC to quit"], base=frame)
        rgb = cv2.cvtColor(cv2.resize(frame, (size[0], int(bc.H * s))), cv2.COLOR_BGR2RGB)
        screen.fill((10, 10, 12))
        screen.blit(pygame.surfarray.make_surface(np.ascontiguousarray(rgb.swapaxes(0, 1))), (0, 0))
        screen.blit(small.render(f"You drive {mt.opp.name} ({ROSTER[robot].blurb})  ·  {HELP}", True, (220, 220, 220)),
                    (8, int(bc.H * s) + 6))
        pygame.display.flip()
        # real-time pacing (the sim never runs ahead of the wall clock)
        lag = mt.t - (time.time() - clock_wall)
        if lag > 0:
            time.sleep(lag)
        if done:
            waiting = True
            while waiting and running:
                for ev in pygame.event.get():
                    if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                        running = False
                    elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_r:
                        waiting = False
                time.sleep(0.03)
            if running:
                bc.close(); mt.close()
                mt, st = new_match()
                bc = Broadcast(mt, h=540, w_ai=540, w_tv=720)
                clock_wall = time.time()
    bc.close()
    mt.close()
    pygame.quit()
    _ = math
