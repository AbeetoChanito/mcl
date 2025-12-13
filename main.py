from config import *
import robot
import pygame
import math
import sys

# ---------------- INIT ----------------

pygame.init()
screen = pygame.display.set_mode((SCREEN_SIZE, SCREEN_SIZE))
pygame.display.set_caption("MCL Simulation")

clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 24)

r = robot.Robot(
    x=0,
    y=0,
    w=ROBOT_WIDTH,
    h=ROBOT_HEIGHT,
    theta=0,
    sensors=[
        ("Front", ROBOT_WIDTH / 2, 0, 0),
        ("Back", -ROBOT_WIDTH / 2, 0, math.pi),
        ("Left", 0, ROBOT_HEIGHT / 2, math.pi / 2),
        ("Right", 0, -ROBOT_HEIGHT / 2, -math.pi / 2),
    ],
)

# ---------------- MAIN LOOP ----------------

running = True
while running:
    dt = clock.tick(FPS) / 1000.0

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            running = False

    keys = pygame.key.get_pressed()

    speed = 40.0  # inches per second
    turn = 3.0  # rad/s

    r.v = 0.0
    r.omega = 0.0

    if keys[pygame.K_UP]:
        r.v += speed
    if keys[pygame.K_DOWN]:
        r.v -= speed
    if keys[pygame.K_LEFT]:
        r.omega += turn
    if keys[pygame.K_RIGHT]:
        r.omega -= turn

    r.update(dt)

    # ---------------- DRAW ----------------

    screen.fill((240, 240, 240))

    # Field border
    top_x, top_y = world_to_screen(-FIELD_SIZE / 2, FIELD_SIZE / 2)
    bottom_x, bottom_y = world_to_screen(FIELD_SIZE / 2, -FIELD_SIZE / 2)
    pygame.draw.rect(screen, (0, 0, 0), (top_x, top_y, bottom_x - top_x, bottom_y - top_y), 3)

    # Robot
    r.draw(screen, font)

    pygame.display.flip()

pygame.quit()
sys.exit()
