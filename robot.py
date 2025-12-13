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

        self.left_wheel_delta = 0.0
        self.right_wheel_delta = 0.0

        self.noisy_odom_x = 0
        self.noisy_odom_y = 0

    def update_rays(self):
        for sensor in self.sensors:
            self.rays[sensor.name] = sensor.cast_sensor(self)

    def update(self, dt):
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt
        self.update_rays()

        # noisy odom
        self.left_wheel_delta, self.right_wheel_delta = self.get_wheel_deltas(dt)

        s = (self.left_wheel_delta + self.right_wheel_delta) / 2.0
        self.noisy_odom_x += s * math.cos(self.theta)
        self.noisy_odom_y += s * math.sin(self.theta)

    def corners(self):
        hw, hh = self.w / 2, self.h / 2
        local_corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
        c, s = math.cos(self.theta), math.sin(self.theta)
        return [
            (self.x + px * c - py * s, self.y + px * s + py * c)
            for px, py in local_corners
        ]
    
    def get_wheel_deltas(self, dt):
        left_wheel_delta = (self.v - self.omega * self.w / 2) * dt
        right_wheel_delta = (self.v + self.omega * self.w / 2) * dt

        left_wheel_delta += random.gauss(0, WHEEL_NOISE_STD)
        right_wheel_delta += random.gauss(0, WHEEL_NOISE_STD)

        return left_wheel_delta, right_wheel_delta

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

        pygame.draw.circle(screen, (200, 0, 0), world_to_screen(self.x, self.y), 5)
        pygame.draw.circle(screen, (0, 200, 0), world_to_screen(self.noisy_odom_x, self.noisy_odom_y), 5)

        text = font.render(f"Left Wheel Delta: {self.left_wheel_delta:.4f} in", True, (0, 0, 0))
        screen.blit(text, (10, y_text))
        y_text += 20
        text = font.render(f"Right Wheel Delta: {self.right_wheel_delta:.4f} in", True, (0, 0, 0))
        screen.blit(text, (10, y_text)) 