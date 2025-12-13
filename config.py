FIELD_SIZE = 144.0
SCALE = 4
SCREEN_SIZE = 800
MAX_RAY = 300.0
FPS = 60
SENSOR_NOISE_STD = 1.0
ROBOT_WIDTH = 18.0
ROBOT_HEIGHT = 15.0

def world_to_screen(x, y):
    return (
        int((x + FIELD_SIZE / 2) * SCALE) + SCREEN_SIZE / 2 - FIELD_SIZE / 2 * SCALE,
        int((FIELD_SIZE / 2 - y) * SCALE) + SCREEN_SIZE / 2 - FIELD_SIZE / 2 * SCALE,
    )
