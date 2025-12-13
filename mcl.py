import random
from config import *
import math
import copy
import pygame

class Particle:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = 1.0

    def update_bayesian(self, sensors):
        self.weight = 1.0
        for sensor in sensors:
            self.weight *= sensor.p(self)

    def out_of_field(self):
        HALF = FIELD_SIZE / 2
        return not (-HALF <= self.x <= HALF and -HALF <= self.y <= HALF)

class MCL:
    @staticmethod
    def update_noisy(particle, left, right, theta):
        s_noisy = (left + right) / 2 + random.gauss(0, WHEEL_NOISE_STD / math.sqrt(2))

        particle.x += s_noisy * math.cos((theta + particle.theta) / 2)
        particle.y += s_noisy * math.sin((theta + particle.theta) / 2)
        particle.theta = theta

    def __init__(self):
        self.particles = [
            Particle(0, 0, 0.0) for _ in range(500)
        ]
        self.prediction = (0.0, 0.0, 0.0)

    def update(self, robot):
        for p in self.particles:
            self.update_noisy(
                p,
                robot.left_wheel_delta,
                robot.right_wheel_delta,
                robot.theta
            )

        for p in self.particles:
            p.update_bayesian(robot.sensors)

        distance_since_update = (robot.left_wheel_delta + robot.right_wheel_delta) / 2

        if distance_since_update < 1e-3:
            x_sum = 0
            y_sum = 0

            for p in self.particles:
                x_sum += p.x
                y_sum += p.y

            self.prediction = (x_sum / len(self.particles), y_sum / len(self.particles), robot.theta)
            return
        
        weight_sum = 0.0

        for p in self.particles:
            if p.out_of_field():
                p.x = random.uniform(-FIELD_SIZE / 2, FIELD_SIZE / 2)
                p.y = random.uniform(-FIELD_SIZE / 2, FIELD_SIZE / 2)

            p.update_bayesian(robot.sensors)
            weight_sum += p.weight

        L = len(self.particles)
        avg_weight = weight_sum / L
        rand_weight = random.uniform(0.0, avg_weight)
        old_particles = copy.deepcopy(self.particles)

        j = 0
        weight_sum = 0.0

        x_sum = 0.0
        y_sum = 0.0

        for i in range(len(self.particles)):
            target = i * avg_weight + rand_weight

            while weight_sum < target:
                if j >= L:
                    break
                weight_sum += old_particles[j].weight
                j += 1

            self.particles[i] = copy.deepcopy(old_particles[j - 1])

            x_sum += self.particles[i].x
            y_sum += self.particles[i].y

        self.prediction = (x_sum / L, y_sum / L, robot.theta)

    def draw_particles(self, screen):
        for p in self.particles:
            pygame.draw.circle(screen, (0, 0, 0), world_to_screen(p.x, p.y), 2)
