from config import *
import math
import random


def ray_to_wall(x, y, dx, dy):
    HALF = FIELD_SIZE / 2
    hits = []

    if dx != 0:
        for X in (HALF, -HALF):
            t = (X - x) / dx
            if t > 0:
                yh = y + t * dy
                if -HALF <= yh <= HALF:
                    hits.append(t)

    if dy != 0:
        for Y in (HALF, -HALF):
            t = (Y - y) / dy
            if t > 0:
                xh = x + t * dx
                if -HALF <= xh <= HALF:
                    hits.append(t)

    return min(hits) if hits else MAX_RAY


def cast_sensor(robot, ox, oy, angle_offset):
    c, s = math.cos(robot.theta), math.sin(robot.theta)

    sx = robot.x + ox * c - oy * s
    sy = robot.y + ox * s + oy * c

    th = robot.theta + angle_offset
    dx, dy = math.cos(th), math.sin(th)

    d_true = min(ray_to_wall(sx, sy, dx, dy), MAX_RAY)

    d_noisy = d_true + random.gauss(0, SENSOR_NOISE_STD)

    ex = sx + dx * d_true
    ey = sy + dy * d_true

    return (sx, sy), (ex, ey), d_noisy
