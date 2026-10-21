import pygame
import random
import math
import sys

pygame.init()

# Dimensions de la fenêtre
WIDTH, HEIGHT = 900, 650
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Simulation de Nuée (Boids Algorithm)")
clock = pygame.time.Clock()

# Couleurs (Thème Néon & Espace Sombre)
DARK_BG = (12, 15, 22)
BOID_COLOR = (0, 255, 157)   # Vert néon
OBSTACLE_COLOR = (255, 75, 75) # Rouge
WHITE = (255, 255, 255)
GRAY = (120, 130, 150)

# Polices
font_medium = pygame.font.SysFont("Arial", 20, bold=True)
font_small = pygame.font.SysFont("Arial", 15)

class Boid:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        # Vitesse et direction aléatoires
        angle = random.uniform(0, 2 * math.pi)
        speed = random.uniform(2, 4)
        self.vx = math.cos(angle) * speed
        self.vy = math.sin(angle) * speed
        self.max_speed = 4.5
        self.max_force = 0.15
        self.radius = 4

    def update(self):
        self.x += self.vx
        self.y += self.vy

        # Gestion des bords de l'écran (effet tore / passage de l'autre côté)
        if self.x < 0: self.x = WIDTH
        elif self.x > WIDTH: self.x = 0
        if self.y < 0: self.y = HEIGHT
        elif self.y > HEIGHT: self.y = 0

    def apply_behavior(self, boids, obstacles):
        # 3 règles de Craig Reynolds + évitement d'obstacles
        sep = self.separation(boids)
        ali = self.alignment(boids)
        coh = self.cohesion(boids)
        obs = self.avoid_obstacles(obstacles)

        # Pondération des forces
        self.vx += sep[0] * 1.5 + ali[0] * 1.0 + coh[0] * 1.0 + obs[0] * 2.0
        self.vy += sep[1] * 1.5 + ali[1] * 1.0 + coh[1] * 1.0 + obs[1] * 2.0

        # Limitation de la vitesse maximale
        speed = math.sqrt(self.vx**2 + self.vy**2)
        if speed > self.max_speed:
            self.vx = (self.vx / speed) * self.max_speed
            self.vy = (self.vy / speed) * self.max_speed

    def limit_force(self, force_x, force_y):
        mag = math.sqrt(force_x**2 + force_y**2)
        if mag > self.max_force:
            force_x = (force_x / mag) * self.max_force
            force_y = (force_y / mag) * self.max_force
        return force_x, force_y

    def separation(self, boids):
        steer_x, steer_y = 0, 0
        count = 0
        desired_separation = 25
        for other in boids:
            dist = math.hypot(self.x - other.x, self.y - other.y)
            if 0 < dist < desired_separation:
                dx = self.x - other.x
                dy = self.y - other.y
                dx /= dist  # Normalisation
                dy /= dist
                steer_x += dx
                steer_y += dy
                count += 1
        if count > 0:
            steer_x /= count
            steer_y /= count
            mag = math.sqrt(steer_x**2 + steer_y**2)
            if mag > 0:
                steer_x = (steer_x / mag) * self.max_speed - self.vx
                steer_y = (steer_y / mag) * self.max_speed - self.vy
                steer_x, steer_y = self.limit_force(steer_x, steer_y)
        return steer_x, steer_y

    def alignment(self, boids):
        sum_vx, sum_vy = 0, 0
        count = 0
        neighbor_dist = 50
        for other in boids:
            dist = math.hypot(self.x - other.x, self.y - other.y)
            if 0 < dist < neighbor_dist:
                sum_vx += other.vx
                sum_vy += other.vy
                count += 1
        if count > 0:
            sum_vx /= count
            sum_vy /= count
            mag = math.sqrt(sum_vx**2 + sum_vy**2)
            if mag > 0:
                sum_vx = (sum_vx / mag) * self.max_speed
                sum_vy = (sum_vy / mag) * self.max_speed
                steer_x = sum_vx - self.vx
                steer_y = sum_vy - self.vy
                steer_x, steer_y = self.limit_force(steer_x, steer_y)
                return steer_x, steer_y
        return 0, 0

    def cohesion(self, boids):
        sum_x, sum_y = 0, 0
        count = 0
        neighbor_dist = 50
        for other in boids:
            dist = math.hypot(self.x - other.x, self.y - other.y)
            if 0 < dist < neighbor_dist:
                sum_x += other.x
                sum_y += other.y
                count += 1
        if count > 0:
            sum_x /= count
            sum_y /= count
            return self.seek(sum_x, sum_y)
        return 0, 0

    def seek(self, target_x, target_y):
        desired_x = target_x - self.x
        desired_y = target_y - self.y
        mag = math.sqrt(desired_x**2 + desired_y**2)
        if mag > 0:
            desired_x = (desired_x / mag) * self.max_speed
            desired_y = (desired_y / mag) * self.max_speed
            steer_x = desired_x - self.vx
            steer_y = desired_y - self.vy
            return self.limit_force(steer_x, steer_y)
        return 0, 0

    def avoid_obstacles(self, obstacles):
        steer_x, steer_y = 0, 0
        for obs in obstacles:
            dist = math.hypot(self.x - obs["x"], self.y - obs["y"])
            if dist < 60: # Rayon de détection d'obstacle
                dx = self.x - obs["x"]
                dy = self.y - obs["y"]
                steer_x += dx / (dist + 0.1)
                steer_y += dy / (dist + 0.1)
        return steer_x, steer_y

# Initialisation de la nuée et des obstacles
boids = [Boid(random.randint(100, WIDTH-100), random.randint(100, HEIGHT-100)) for _ in range(70)]
obstacles = []

running = True
while running:
    mouse_pos = pygame.mouse.get_pos()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # Clic gauche : Ajouter un obstacle
                obstacles.append({"x": mouse_pos[0], "y": mouse_pos[1], "radius": 15})
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_c:
                obstacles = []  # Effacer les obstacles
            elif event.key == pygame.K_r:
                boids = [Boid(random.randint(100, WIDTH-100), random.randint(100, HEIGHT-100)) for _ in range(70)]

    screen.fill(DARK_BG)

    # 1. Mise à jour et dessin des obstacles
    for obs in obstacles:
        pygame.draw.circle(screen, OBSTACLE_COLOR, (obs["x"], obs["y"]), obs["radius"])
        pygame.draw.circle(screen, WHITE, (obs["x"], obs["y"]), obs["radius"], 2)

    # 2. Mise à jour et dessin des boids
    for b in boids:
        b.apply_behavior(boids, obstacles)
        b.update()

        # Dessin du boid sous forme de petit triangle orienté dans le sens du mouvement
        angle = math.atan2(b.vy, b.vx)
        p1 = (b.x + math.cos(angle) * 10, b.y + math.sin(angle) * 10)
        p2 = (b.x + math.cos(angle + 2.5) * 6, b.y + math.sin(angle + 2.5) * 6)
        p3 = (b.x + math.cos(angle - 2.5) * 6, b.y + math.sin(angle - 2.5) * 6)
        pygame.draw.polygon(screen, BOID_COLOR, [p1, p2, p3])

    # 3. En-tête et Instructions
    title_txt = font_medium.render("SIMULATION DE NUÉE (BOIDS - INTELLIGENCE COLLECTIVE)", True, WHITE)
    screen.blit(title_txt, (WIDTH // 2 - title_txt.get_width() // 2, 20))

    controls_txt = font_small.render("CLIC GAUCHE : Placer un obstacle | C : Effacer les obstacles | R : Réinitialiser la nuée", True, GRAY)
    screen.blit(controls_txt, (WIDTH // 2 - controls_txt.get_width() // 2, 52))

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
sys.exit()