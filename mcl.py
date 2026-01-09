import random
from config import *
import math
import copy
import pygame

MAG_GAIN = 0.25
MIN_GAIN = 0.05

class Particle:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = 1.0
    
    def update_bayesian(self, sensors):
        self.weight = 1.0
        for sensor in sensors:
            likelihood = sensor.p(self)
            if likelihood is not None:
                self.weight *= likelihood
    
    def out_of_field(self):
        HALF = FIELD_SIZE / 2
        return not (-HALF <= self.x <= HALF and -HALF <= self.y <= HALF)


class MCL:
    def __init__(self):
        self.particles = [
            Particle(0.0, 0.0, 0.0) for _ in range(500)
        ]
        self.prediction = (0.0, 0.0, 0.0)
        self.previous_theta = None
    
    def set_pose(self, x, y, theta):
        """Initialize all particles to a specific pose"""
        for p in self.particles:
            p.x = x
            p.y = y
            p.theta = theta
            p.weight = 1.0
        self.prediction = (x, y, theta)
        self.previous_theta = theta
    
    def get_odo_delta(self, l_delta, r_delta, theta):
        """Calculate odometry delta using average theta"""
        if self.previous_theta is not None:
            true_theta = (theta + self.previous_theta) / 2.0
        else:
            true_theta = theta
        
        s = (l_delta + r_delta) / 2.0
        dx = s * math.cos(true_theta)
        dy = s * math.sin(true_theta)
        
        return dx, dy
    
    def motion_update(self, l_delta, r_delta, current_theta):
        """Update particle positions with noise based on movement magnitude"""
        dx, dy = self.get_odo_delta(l_delta, r_delta, current_theta)
        odo_hypot = math.hypot(dx, dy)
        
        std = max(MAG_GAIN * odo_hypot, MIN_GAIN)
        
        for p in self.particles:
            noise_x = random.gauss(0, std)
            noise_y = random.gauss(0, std)
            p.x += dx + noise_x
            p.y += dy + noise_y
            p.theta = current_theta 
        
        self.previous_theta = current_theta
        
        return odo_hypot
    
    def update(self, robot, dt):
        displacement = self.motion_update(
            robot.left_wheel_delta,
            robot.right_wheel_delta,
            robot.theta
        )
        
        weight_sum = 0.0
        for p in self.particles:
            if p.out_of_field():
                p.x = random.uniform(-FIELD_SIZE / 2, FIELD_SIZE / 2)
                p.y = random.uniform(-FIELD_SIZE / 2, FIELD_SIZE / 2)
                p.theta = robot.theta
            
            p.weight = 1.0
            for sensor in robot.sensors:
                likelihood = sensor.p(p)
                if likelihood is not None:
                    p.weight *= likelihood
            
            weight_sum += p.weight
            
        if weight_sum == 0.0 or displacement < 1e-6:
            sum_x = 0.0
            sum_y = 0.0
            for p in self.particles:
                sum_x += p.x
                sum_y += p.y
            
            self.prediction = (
                sum_x / len(self.particles),
                sum_y / len(self.particles),
                robot.theta
            )
            return
        
        new_particles = []
        L = len(self.particles)
        avg_weight = weight_sum / L
        
        r = random.uniform(0.0, avg_weight)
        c = self.particles[0].weight
        i = 0
        
        for m in range(L):
            target = r + m * avg_weight
            while target > c and i < L - 1:
                i += 1
                c += self.particles[i].weight
            new_particles.append(copy.deepcopy(self.particles[i]))
        
        self.particles = new_particles
        
        sum_x = 0.0
        sum_y = 0.0
        for p in self.particles:
            sum_x += p.x
            sum_y += p.y
        
        self.prediction = (
            sum_x / L,
            sum_y / L,
            robot.theta
        )
    
    def draw_particles(self, screen):
        for p in self.particles:
            pygame.draw.circle(screen, (0, 0, 0), world_to_screen(p.x, p.y), 2)