# TODO
# - pb quand on part du coin en haut à gauche
# - implementer une limitation en acceleration
# - implementer des rampes d'accelération

# - definir vitesse max par roue et lineaire (x et y combines)

from icecream import ic
import pygame
import sys
import time
from math import *
import numpy as np
from pid import PID
import pygame_widgets as pw
from pygame_widgets.button import Button

pygame.init()

# CONSTANTS
WIDTH, HEIGHT = 900, 600  # window
TABLE_WIDTH, TABLE_HEIGHT = 300, 200  # Table size in cm
FPS = 50

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Simulateur Robot Holonome")

# COLORS
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
PINK = (255, 0, 255)


# ROBOT


class Robot():
    def __init__(self) -> None:
        self.pos = np.array([TABLE_WIDTH/2,  # x, m
                             TABLE_HEIGHT/2,  # y, m
                             0])  # theta, rad between -pi and pi
        self.rayon_roue = 5.8
        self.epaisseur_roue = 5
        self.rayon_robot = 15
        self.robot_positions = [self.pos]

        self.vitesse_lineaire = [0, 0]  # m/s
        self.vitesse_angulaire = 0  # rad/s
        self.has_finished_rotation = False

        self.vitesse_roue0 = 0
        self.vitesse_roue1 = 0
        self.vitesse_roue2 = 0

        self.current_goal_index = -1
        self.current_goal = self.pos
        self.goal_reached = True

        self.max_lin_speed = 50  # cm/s
        self.max_ang_speed = 1  # pi/2  # rad/s

        # juste un P, normal pour la position
        self.pid_pos_x = PID(10.0, 0, 0, 1/FPS)
        self.pid_pos_y = PID(10.0, 0, 0, 1/FPS)
        self.pid_pos_theta = PID(50, 0, 0, 1/FPS)

    def goto(self, x, y, theta):
        # si on a pas fini la rotation, on continue à tourner
        # sinon on fait la translation
        if not self.has_finished_rotation:
            # si c'est plus court de tourner dans l'autre sens, on tourne dans l'autre sens
            error_theta = theta - self.pos[2]
            if error_theta > pi:
                error_theta -= 2*pi
            elif error_theta < -pi:
                error_theta += 2*pi
            self.vitesse_angulaire = self.pid_pos_theta.update(error_theta)
            # limit the speeds (speeds can be negative)
            self.vitesse_angulaire = max(-self.max_ang_speed,
                                         min(self.vitesse_angulaire, self.max_ang_speed))
            # print("error_theta", error_theta,
            #       "vitesse_angulaire", self.vitesse_angulaire)
            if self.check_rotation_reached():
                self.has_finished_rotation = True
                self.vitesse_angulaire = 0
        # else:
            # # PID
            self.vitesse_lineaire = [
                self.pid_pos_x.update(x - self.pos[0]),
                self.pid_pos_y.update(y - self.pos[1])
            ]
            # vitesse_lineaire_projetee = [
            #     self.vitesse_lineaire[0]*cos(self.pos[2]) -
            #     self.vitesse_lineaire[1]*sin(self.pos[2]),
            #     self.vitesse_lineaire[0]*sin(self.pos[2]) +
            #     self.vitesse_lineaire[1]*cos(self.pos[2])
            # ]
            # self.vitesse_lineaire = vitesse_lineaire_projetee

            # print("vitesse_lineaire_projetee", vitesse_lineaire_projetee,
            #       "vitese_lin", self.vitesse_lineaire)

            # # limit the speeds (speeds can be negative)
            # self.vitesse_lineaire = [
            #     max(-self.max_lin_speed,
            #         min(vitesse_lineaire_projetee[0], self.max_lin_speed)),
            #     max(-self.max_lin_speed,
            #         min(vitesse_lineaire_projetee[1], self.max_lin_speed))
            # ]

    def goto_next_goal(self):
        self.current_goal_index += 1
        self.goal_reached = False
        self.has_finished_rotation = False
        self.current_goal = goals_positions[self.current_goal_index]

    def check_angle(self, angle1, angle2, error_max):
        error = abs(self.pos[2] - self.current_goal[2])
        if (abs(2*pi-error) < 0.01):
            error = 0
        return error < error_max

    def check_goal_reached(self):
        error_max_lin = 1
        error_max_ang = 0.01

        if abs(self.pos[0] - self.current_goal[0]) < error_max_lin and abs(self.pos[1] - self.current_goal[1]) < error_max_lin:  # and
            # self.check_angle(self.pos[2], self.current_goal[2], error_max_ang):
            if (self.current_goal_index < len(goals_positions)-1):
                self.current_goal_index += 1
            # self.current_goal_index %= len(goals_positions)
            self.goal_reached = True

    def check_rotation_reached(self):
        error_max_ang = 0.01
        return self.check_angle(self.pos[2], self.current_goal[2], error_max_ang)

    def update_robot_position(self):
        global FPS
        self.goto(self.current_goal[0],
                  self.current_goal[1],
                  self.current_goal[2])
        self.vitesses_to_roues()

        # Convertir les vitesses angulaires des roues en vitesse linéaire et angulaire du robot
        self.vitesse_lineaire = [-(self.vitesse_roue0 + self.vitesse_roue1 - 2*self.vitesse_roue2),
                                 1/3*(-self.vitesse_roue0*sqrt(3) + self.vitesse_roue1*sqrt(3))]

        self.vitesse_angulaire = (1 / (self.rayon_robot)) * \
            (-self.vitesse_roue0 - self.vitesse_roue1 + self.vitesse_roue2)

        # Mettre à jour les coordonnées du robot en fonction des vitesses
        delta_t = 1 / FPS  # Temps entre deux mises à jour
        self.pos[0] += self.vitesse_lineaire[0] * delta_t
        self.pos[1] += self.vitesse_lineaire[1] * delta_t
        self.pos[2] += self.vitesse_angulaire * delta_t

        self.robot_positions.append(self.pos.copy())

    def vitesses_to_roues(self):
        # On calcule la vitesse de chaque roue en mètres par seconde
        self.vitesse_roue0 = 0.5 * \
            self.vitesse_lineaire[0] - sqrt(3) / 2 * self.vitesse_lineaire[1] - \
            self.rayon_robot * self.vitesse_angulaire
        self.vitesse_roue1 = 0.5 * \
            self.vitesse_lineaire[0] + sqrt(3) / 2 * self.vitesse_lineaire[1] - \
            self.rayon_robot * self.vitesse_angulaire
        self.vitesse_roue2 = self.vitesse_lineaire[0] - \
            self.rayon_robot * self.vitesse_angulaire        


# Fonction pour convertir les coordonnées réelles en coordonnées écran
def real_to_screen(x, y):
    x_factor = WIDTH / TABLE_WIDTH
    y_factor = HEIGHT / TABLE_HEIGHT
    # offset to center the table
    offset_x = (WIDTH - TABLE_WIDTH / x_factor)*0.9 / 2
    offset_y = (HEIGHT - TABLE_HEIGHT / y_factor)*0.9 / 2
    # scale everything to 0.9
    screen_x = int(x * x_factor*0.9)+45
    screen_y = int(y * y_factor*0.9)+30
    return screen_x, screen_y


def draw_roues(screen, screen_x, screen_y, robot_theta):
    # Dessiner les roues rectangulaires par 4 lignes
    for i in range(3):
        # 120° d'écart entre les roues

        angles_roues = [pi/2 - 4*pi/3,
                        pi/2 - 2*pi/3,
                        pi/2]
        angle_roue = robot_theta + angles_roues[i]

        # Coordonnées du centre de la roue
        centre_roue = (
            screen_x + robot.rayon_robot * cos(angle_roue),
            screen_y + robot.rayon_robot * sin(angle_roue)
        )

        # Coordonnées des coins de la roue
        coin1 = np.array([
            centre_roue[0] + robot.epaisseur_roue/2,
            centre_roue[1] + robot.rayon_roue
        ])

        coin2 = np.array([
            centre_roue[0] + robot.epaisseur_roue/2,
            centre_roue[1] - robot.rayon_roue
        ])

        coin3 = np.array([
            centre_roue[0] - robot.epaisseur_roue/2,
            centre_roue[1] - robot.rayon_roue
        ])

        coin4 = np.array([
            centre_roue[0] - robot.epaisseur_roue/2,
            centre_roue[1] + robot.rayon_roue
        ])
        # now we rotate each point around the center of the wheel
        coin1 = np.array([
            centre_roue[0] + (coin1[0]-centre_roue[0])*cos(angle_roue) -
            (coin1[1]-centre_roue[1])*sin(angle_roue),
            centre_roue[1] + (coin1[0]-centre_roue[0])*sin(angle_roue) +
            (coin1[1]-centre_roue[1])*cos(angle_roue)
        ])
        coin2 = np.array([
            centre_roue[0] + (coin2[0]-centre_roue[0])*cos(angle_roue) -
            (coin2[1]-centre_roue[1])*sin(angle_roue),
            centre_roue[1] + (coin2[0]-centre_roue[0])*sin(angle_roue) +
            (coin2[1]-centre_roue[1])*cos(angle_roue)
        ])
        coin3 = np.array([
            centre_roue[0] + (coin3[0]-centre_roue[0])*cos(angle_roue) -
            (coin3[1]-centre_roue[1])*sin(angle_roue),
            centre_roue[1] + (coin3[0]-centre_roue[0])*sin(angle_roue) +
            (coin3[1]-centre_roue[1])*cos(angle_roue)
        ])
        coin4 = np.array([
            centre_roue[0] + (coin4[0]-centre_roue[0])*cos(angle_roue) -
            (coin4[1]-centre_roue[1])*sin(angle_roue),
            centre_roue[1] + (coin4[0]-centre_roue[0])*sin(angle_roue) +
            (coin4[1]-centre_roue[1])*cos(angle_roue)
        ])

        # Dessiner le polygone représentant la roue
        pygame.draw.polygon(screen, BLACK, [coin1, coin2, coin3, coin4])

        # axes des vecteurs vitesses unitaires de la roue en rouge
        v_x = ((coin2 + coin3)/2 - centre_roue)
        v_x = v_x / np.linalg.norm(v_x)
        v_y = ((coin2 + coin1)/2 - centre_roue)
        v_y = v_y / np.linalg.norm(v_y)
        pygame.draw.line(screen, BLACK, centre_roue, centre_roue + v_x * 20, 2)
        pygame.draw.line(screen, BLACK, centre_roue, centre_roue + v_y * 20, 2)

        # texte numero de roue
        font = pygame.font.SysFont('Arial', 10)
        text = font.render(str(i), True, BLACK)
        textRect = text.get_rect()
        textRect.center = (v_y[0]*10+v_x[0]*10+centre_roue[0],
                           v_y[1]*10+v_x[1]*10+centre_roue[1])
        screen.blit(text, textRect)


def draw():
    # drawings
    screen.fill(WHITE)

    # Dessiner la table
    screen_table_x, screen_table_y = real_to_screen(0, 0)
    screen_table_width, screen_table_height = real_to_screen(
        TABLE_WIDTH, TABLE_HEIGHT)
    bottom_right_corner = (screen_table_width-45, screen_table_height-30)

    pygame.draw.rect(screen, BLACK, (screen_table_x, screen_table_y,
                     bottom_right_corner[0], bottom_right_corner[1]), 3)

    # zones
    zone_size = real_to_screen(45, 45)[0]
    pygame.draw.rect(screen, BLACK, (screen_table_x,
                     screen_table_y, zone_size, zone_size), 3)  # haut gauche
    pygame.draw.rect(screen, BLACK, (screen_table_x+screen_table_width -
                     zone_size, screen_table_y, zone_size, zone_size), 3)  # haut droite
    pygame.draw.rect(screen, BLACK, (screen_table_x, screen_table_y +
                     screen_table_height-zone_size, zone_size, zone_size), 3)  # bas gauche
    pygame.draw.rect(screen, BLACK, (screen_table_x+screen_table_width-zone_size,
                     screen_table_y+screen_table_height-zone_size, zone_size, zone_size), 3)  # bas droite

    # draw circle at right upper corner
    pygame.draw.circle(
        screen, BLACK, (bottom_right_corner[0], screen_table_y), 10, 3)

    # Dessiner les positions du robot
    for pos in robot.robot_positions:
        screen_x, screen_y = real_to_screen(pos[0], pos[1])
        pygame.draw.circle(screen, BLUE, (screen_x, screen_y), 1)

    # draw all next goals by a cross and an arrow
    for i in range(len(goals_positions)):
        screen_x, screen_y = real_to_screen(
            goals_positions[i][0], goals_positions[i][1])
        pygame.draw.line(screen, RED, (screen_x-5, screen_y-5),
                         (screen_x+5, screen_y+5), 2)
        pygame.draw.line(screen, RED, (screen_x+5, screen_y-5),
                         (screen_x-5, screen_y+5), 2)
        # arrow indicating the angle
        pygame.draw.line(screen, RED, (screen_x, screen_y),
                         (screen_x+20*cos(goals_positions[i][2]-pi/2),
                             screen_y+20*sin(goals_positions[i][2]-pi/2)), 2)
        # text
        angle = goals_positions[i][2]*180/pi
        font = pygame.font.SysFont('Arial', 10)
        text = font.render(str(angle)+"°", True, BLACK)
        textRect = text.get_rect()
        textRect.center = (screen_x, screen_y)
        screen.blit(text, textRect)

    # Dessiner le robot à sa nouvelle position
    screen_x, screen_y = real_to_screen(robot.pos[0], robot.pos[1])
    pygame.draw.circle(screen, BLACK, (screen_x, screen_y),
                       robot.rayon_robot, 1)
    pygame.draw.circle(screen, GREEN, (screen_x, screen_y), 1)

    # dessiner la direction du robot
    pygame.draw.line(screen, GREEN, (screen_x, screen_y),
                     (screen_x+20*cos(-pi/2+robot.pos[2]), screen_y+20*sin(-pi/2+robot.pos[2])), 2)

    draw_roues(screen, screen_x, screen_y, robot.pos[2])

    # print the speeds in the top left corner
    font = pygame.font.SysFont('Arial', 20)
    text = font.render(
        "speed_x: " + str(round(robot.vitesse_lineaire[0], 2)) + " cm/s", True, BLACK)
    textRect = text.get_rect()
    textRect.topleft = (50, 30)
    screen.blit(text, textRect)
    text = font.render(
        "speed_y: " + str(round(robot.vitesse_lineaire[1], 2)) + " cm/s", True, BLACK)
    textRect = text.get_rect()
    textRect.topleft = (50, 50)
    screen.blit(text, textRect)
    text = font.render(
        "speed_theta: " + str(round(robot.vitesse_angulaire, 2)) + " rad/s", True, BLACK)
    textRect = text.get_rect()
    textRect.topleft = (50, 70)
    screen.blit(text, textRect)
    text = font.render(
        "theta: " + str(round(robot.pos[2], 2)) + " rad", True, BLACK)
    textRect = text.get_rect()
    textRect.topleft = (50, 90)
    screen.blit(text, textRect)

    button.draw()

    if waiting_for_release:
        # draw a line from pos_waiting to the mouse
        mouse_pos = pygame.mouse.get_pos()
        x, y = mouse_pos
        x, y = x-45, y-30
        x, y = x/(WIDTH-90)*TABLE_WIDTH, y/(HEIGHT-60)*TABLE_HEIGHT
        pygame.draw.line(screen, BLACK, real_to_screen(
            pos_waiting[0], pos_waiting[1]), real_to_screen(x, y), 2)

    # Mettre à jour l'affichage
    pygame.display.flip()


goals_positions = [[TABLE_WIDTH/2, TABLE_HEIGHT/2, 0],
                   ]  # [x, y, theta]
robot = Robot()


def on_click():
    global robot
    print("button clicked")
    robot.vitesse_lineaire = [0, 0]


# Boucle principale
clock = pygame.time.Clock()
button = Button(
    screen, 5, HEIGHT-30-5, 50, 30, text='Pause',
    fontSize=13, margin=20,
    inactiveColour=(100, 100, 100),
    hoverColour=(0, 0, 100),
    radius=20,
    onClick=lambda: on_click()
)

waiting_for_release = False
pos_waiting = []

DEBUG_moved = False

while True:
    events = pygame.event.get()
    for event in events:
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                pygame.quit()
                sys.exit()

        # si un clic souris, on va à la position du clic
        if event.type == pygame.MOUSEBUTTONDOWN:
            mouse_pos = pygame.mouse.get_pos()
            x, y = mouse_pos
            x, y = x-45, y-30
            x, y = x/(WIDTH-90)*TABLE_WIDTH, y/(HEIGHT-60)*TABLE_HEIGHT
            waiting_for_release = True
            pos_waiting = [x, y]

        if event.type == pygame.MOUSEBUTTONUP and waiting_for_release:
            mouse_pos = pygame.mouse.get_pos()
            x, y = mouse_pos
            x, y = x-45, y-30
            x, y = x/(WIDTH-90)*TABLE_WIDTH, y/(HEIGHT-60)*TABLE_HEIGHT
            # angle
            theta = atan2(y-pos_waiting[1], x-pos_waiting[0]) + pi/2
            print("goto", pos_waiting, theta*180/pi)
            goals_positions.append([pos_waiting[0], pos_waiting[1], theta])
            waiting_for_release = False
            DEBUG_moved = True
            robot.goto_next_goal()

        # si clic droit, on supprime tous les goals
        if event.type == pygame.MOUSEBUTTONUP:
            if event.button == 3:
                goals_positions = [[TABLE_WIDTH/2, TABLE_HEIGHT/2, 0]]
    button.listen(events)
    pw.update(events)

    robot.check_goal_reached()
    robot.update_robot_position()

    draw()

    clock.tick(FPS)
    # if DEBUG_moved:
    #     # quit()
    #     pass
