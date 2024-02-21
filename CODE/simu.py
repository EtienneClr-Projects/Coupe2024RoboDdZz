# conda activate myenv (etienne)

from kinematic_models import Robot_Kinematic_Model, Obstacle_static_model
import avoidance

from icecream import ic
import pygame, sys
from math import atan2, pi, cos, sin
import numpy as np
import pygame_widgets as pw
from pygame_widgets.button import Button
from skgeom import Point2, Polygon
from dijkstar import Graph

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


# Fonction pour convertir les coordonnées réelles en coordonnées écran
def real_to_screen(x, y):
    x = float(x)
    y = float(y)
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


def draw_obstacle(screen, obstacle, color):
    vertices = list(obstacle.vertices)
    vertices.append(vertices[0])
    for i in range(len(vertices)-1):
        A = real_to_screen(float(vertices[i].x()),float(vertices[i].y()))
        B = real_to_screen(float(vertices[i+1].x()),float(vertices[i+1].y()))
        pygame.draw.line(screen, color, A, B)

def draw_graph_and_path(screen):
    # draw the graph
    for A,B in graph.items():
        pointA = dico_all_points[A]
        for b in B.keys():
            pointB = dico_all_points[b]
            pygame.draw.line(screen, BLACK, 
                             real_to_screen(float(pointA[0]),float(pointA[1])), 
                             real_to_screen(float(pointB[0]),float(pointB[1])),
                             3)
            
            # font = pygame.font.SysFont('Arial', 20)
            # text = font.render(str(A), True, RED)
            # textRect = text.get_rect()
            # textRect.center = real_to_screen(float(pointA[0]),float(pointA[1]))
            # screen.blit(text, textRect)
            

    # draw path
    if path is not None:
        nodes = path
        for i in range(len(nodes)-1):
            pygame.draw.line(screen, GREEN, 
                real_to_screen(dico_all_points[nodes[i]][0],dico_all_points[nodes[i]][1]), 
                real_to_screen(dico_all_points[nodes[i+1]][0], dico_all_points[nodes[i+1]][1]), 
                5)

    # draw the obstacle
    draw_obstacle(screen, obstacle.polygon, RED)    
    # draw_obstacle(screen, obstacle.expanded_obstacle_poly, YELLOW)  

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

    # draw robot obstacle
    # draw_obstacle(screen, obstacle)

    # Dessiner les positions du robot
    for pos in robot.robot_positions:
        screen_x, screen_y = real_to_screen(pos[0], pos[1])
        pygame.draw.circle(screen, BLUE, (screen_x, screen_y), 1)

    # draw all next goals by a cross and an arrow
    for i in range(len(robot.goals_positions)):
        screen_x, screen_y = real_to_screen(
            robot.goals_positions[i][0], robot.goals_positions[i][1])
        pygame.draw.line(screen, GREEN, (screen_x-5, screen_y-5),
                         (screen_x+5, screen_y+5), 2)
        pygame.draw.line(screen, GREEN, (screen_x+5, screen_y-5),
                         (screen_x-5, screen_y+5), 2)
        # arrow indicating the angle
        pygame.draw.line(screen, GREEN, (screen_x, screen_y),
                         (screen_x+20*cos(robot.goals_positions[i][2]-pi/2),
                             screen_y+20*sin(robot.goals_positions[i][2]-pi/2)), 2)
        # text
        angle = robot.goals_positions[i][2]*180/pi
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

    draw_graph_and_path(screen)

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
    pygame.display.flip() # TODO remettre

def on_click():
    global robot
    print("button clicked")
    robot.vitesse_lineaire = [0, 0]


robot = Robot_Kinematic_Model(TABLE_WIDTH=TABLE_WIDTH, TABLE_HEIGHT=TABLE_HEIGHT,FPS=FPS)
offset = 10
obstacle = Obstacle_static_model(center_x=100, center_y= 100, width= 10, height= 10,offset=offset)


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


graph = Graph()
path = None

# Boucle principale
while True:
    # EVENTS
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
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1: # left click down
            mouse_pos = pygame.mouse.get_pos()
            x, y = mouse_pos
            x, y = x-45, y-30
            x, y = x/(WIDTH-90)*TABLE_WIDTH, y/(HEIGHT-60)*TABLE_HEIGHT
            waiting_for_release = True
            pos_waiting = [x, y]

        if event.type == pygame.MOUSEBUTTONUP and waiting_for_release and event.button == 1: # left click up
            # calcule la position sur la table
            mouse_pos = pygame.mouse.get_pos()
            x, y = mouse_pos
            x, y = x-45, y-30
            x, y = x/(WIDTH-90)*TABLE_WIDTH, y/(HEIGHT-60)*TABLE_HEIGHT
            # calcule l'angle demandé
            theta = atan2(y-pos_waiting[1], x-pos_waiting[0]) + pi/2
            
            # GOTO
            ic("goto", pos_waiting, theta*180/pi)
            waiting_for_release = False

            robot.goals_positions.append([pos_waiting[0],pos_waiting[1],theta])

        # si clic droit on bouge le robot adverse
        if pygame.mouse.get_pressed()[2]:
            if event.type == pygame.MOUSEMOTION or event.type == pygame.MOUSEBUTTONDOWN or event.type == pygame.MOUSEBUTTONUP:
                # if event.button == 3: # right click
                mouse_pos = pygame.mouse.get_pos()
                x, y = mouse_pos
                x, y = x-45, y-30
                x, y = x/(WIDTH-90)*TABLE_WIDTH, y/(HEIGHT-60)*TABLE_HEIGHT
                obstacle = Obstacle_static_model(x,y,10,10,offset)

    button.listen(events)
    pw.update(events)
    # delete the last point so that the array does not get to heavy
    if len(robot.robot_positions)>500:
        robot.robot_positions.pop(0)


    # recompute path
    if len(robot.goals_positions) > 0:
        theta = robot.goals_positions[-1][2] # use the theta of the goal for each point

        start = Point2(robot.pos[0],robot.pos[1])
        goal = Point2(robot.goals_positions[-1][0],robot.goals_positions[-1][1])
        graph, dico_all_points = avoidance.create_graph(start, goal, obstacle.expanded_obstacle_poly)
        path = avoidance.find_avoidance_path(graph, 0, 1).nodes # mais en soit renvoie aussi le coût
        goals = []
        for p in path[1:]: # we don't add the start point
            goals.append([float(dico_all_points[p][0]),float(dico_all_points[p][1]), theta])
        robot.goals_positions = goals


    robot.check_goal_reached()
    robot.update_robot_position()

    draw()    
    clock.tick(FPS)