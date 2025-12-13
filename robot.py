import math
import pygame
from config import *
from ray import *


class Robot:
    def __init__(self, x, y, w, h, theta, sensors):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.theta = theta

        self.v = 0.0
        self.omega = 0.0

        self.sensors = sensors
        self.rays = {}

    def update_rays(self):
        for name, ox, oy, ang in self.sensors:
            self.rays[name] = cast_sensor(self, ox, oy, ang)

    def update(self, dt):
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt
        self.update_rays()

    def corners(self):
        hw, hh = self.w / 2, self.h / 2
        local_corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
        c, s = math.cos(self.theta), math.sin(self.theta)
        return [
            (self.x + px * c - py * s, self.y + px * s + py * c)
            for px, py in local_corners
        ]

    def draw(self, screen, font):
        pygame.draw.polygon(
            screen, (50, 100, 220), [world_to_screen(x, y) for x, y in self.corners()]
        )

        x_text, y_text = 10, 10
        for label, ((sx, sy), (ex, ey), d) in self.rays.items():
            pygame.draw.line(
                screen,
                (220, 50, 50),
                world_to_screen(sx, sy),
                world_to_screen(ex, ey),
                2
            )

            text = font.render(
                f"{label}: {d:.1f} in",
                True,
                (0, 0, 0)
            )
            screen.blit(text, (x_text, y_text))
            y_text += 20
