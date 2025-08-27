import pygame
import math
import serial

# --- Serial Setup ---
ser = serial.Serial('COM5', 9600, timeout=1)  # Change COM port as needed

# --- Pygame Setup ---
pygame.init()
WIDTH, HEIGHT = 800, 800
CENTER = (WIDTH//2, HEIGHT-50)
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Radar Simulation")

clock = pygame.time.Clock()

# Colors
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
DARK_GREEN = (0, 100, 0)
RED = (255, 0, 0)

# --- Draw Radar Grid ---
def draw_grid():
    screen.fill(BLACK)
    # Draw concentric circles
    for r in range(100, 501, 100):
        pygame.draw.circle(screen, GREEN, CENTER, r, 1)
    # Draw radial lines
    for angle in range(0, 181, 30):
        rad = math.radians(angle)
        x = CENTER[0] + 500 * math.cos(rad)
        y = CENTER[1] - 500 * math.sin(rad)
        pygame.draw.line(screen, GREEN, CENTER, (x, y), 1)

# --- Convert Polar to Cartesian ---
def polar_to_cartesian(angle, dist):
    rad = math.radians(angle)
    x = CENTER[0] + dist * math.cos(rad)
    y = CENTER[1] - dist * math.sin(rad)
    return int(x), int(y)

running = True
sweep_angle = 0
points = []

while running:
    clock.tick(60)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    draw_grid()

    # Read from Arduino
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode().strip()
            if line:
                angle, distance = map(int, line.split(","))
                if distance < 200:  # valid object
                    x, y = polar_to_cartesian(angle, distance*2)  # scale for screen
                    points.append((x, y))
                    pygame.draw.circle(screen, RED, (x, y), 5)
        except:
            pass

    # Draw sweeping arc
    sweep_surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
    pygame.draw.polygon(
        sweep_surface, (0, 255, 0, 80), 
        [CENTER, 
         polar_to_cartesian(sweep_angle-2, 500), 
         polar_to_cartesian(sweep_angle+2, 500)]
    )
    screen.blit(sweep_surface, (0, 0))

    # Update sweep angle
    sweep_angle = (sweep_angle + 2) % 181

    pygame.display.flip()

pygame.quit()
