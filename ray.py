from config import *
import math
import random

def standard_normal_pdf(x):
    return (1.0 / math.sqrt(2 * math.pi)) * math.exp(-0.5 * x * x)

class Distance:
    def __init__(self, name, x, y, angle_offset):
        self.name = name
        self.x = x
        self.y = y
        self.angle_offset = angle_offset

        self.sx = 0
        self.sy = 0
        self.ex = 0
        self.ey = 0

        self.d_true = 0
        self.d_noisy = 0

    @staticmethod
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
    
    def update(self, robot):
        c, s = math.cos(robot.theta), math.sin(robot.theta)

        self.sx = robot.x + self.x * c - self.y * s
        self.sy = robot.y + self.x * s + self.y * c

        th = robot.theta + self.angle_offset
        dx, dy = math.cos(th), math.sin(th)

        self.d_true = min(self.ray_to_wall(self.sx, self.sy, dx, dy), MAX_RAY)
        self.d_noisy = self.d_true + random.gauss(0, DISTANCE_NOISE_STD)

        self.ex = self.sx + dx * self.d_true
        self.ey = self.sy + dy * self.d_true

    def p(self, particle):
        c, s = math.cos(particle[2]), math.sin(particle[2])

        sx = particle[0] + self.x * c - self.y * s
        sy = particle[1] + self.x * s + self.y * c

        th = particle[2] + self.angle_offset
        dx, dy = math.cos(th), math.sin(th)

        d = min(self.ray_to_wall(sx, sy, dx, dy), MAX_RAY)

        return standard_normal_pdf((d - self.d_noisy) / DISTANCE_NOISE_STD)
        