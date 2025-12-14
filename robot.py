import math
import pygame
from config import *
from ray import *
from mcl import MCL


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

        self.left_wheel_delta = 0.0
        self.right_wheel_delta = 0.0 
        self.mcl = MCL()

    def update(self, dt):
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt
        self.update_gaussian(dt)
        self.update_rays()
        self.mcl.update(self)

    def update_rays(self):
        for sensor in self.sensors:
            sensor.update(self)
    
    def update_gaussian(self, dt):
        self.left_wheel_delta = (self.v - self.omega * self.h / 2) * dt + random.gauss(0, WHEEL_NOISE_STD)
        self.right_wheel_delta = (self.v + self.omega * self.h / 2) * dt + random.gauss(0, WHEEL_NOISE_STD)

    # ---------------- DRAWING FUNCTIONS ----------------

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
        for sensor in self.sensors:
            pygame.draw.line(
                screen,
                (220, 50, 50),
                world_to_screen(sensor.sx, sensor.sy),
                world_to_screen(sensor.ex, sensor.ey),
                2
            )

            text = font.render(
                f"{sensor.name}: {sensor.d_noisy:.1f} in",
                True,
                (0, 0, 0)
            )
            screen.blit(text, (x_text, y_text))
            y_text += 20

        pygame.draw.circle(screen, (200, 0, 0), world_to_screen(self.x, self.y), 5)
        pygame.draw.circle(screen, (0, 200, 0), world_to_screen(self.mcl.prediction[0], self.mcl.prediction[1]), 5)
        self.mcl.draw_particles(screen)

        text = font.render(f"Left Wheel Delta: {self.left_wheel_delta:.4f} in", True, (0, 0, 0))
        screen.blit(text, (10, y_text))
        y_text += 20
        text = font.render(f"Right Wheel Delta: {self.right_wheel_delta:.4f} in", True, (0, 0, 0))
        screen.blit(text, (10, y_text)) 