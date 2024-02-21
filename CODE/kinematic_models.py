from pid import PID

from typing import List
from math import pi, sqrt
from icecream import ic
from skgeom import Point2, Polygon
import math_bind

class Robot_Kinematic_Model():
    def __init__(self, TABLE_WIDTH, TABLE_HEIGHT, FPS) -> None:
        self.pos = [TABLE_WIDTH/2,  # x, m
                    TABLE_HEIGHT/2,  # y, m
                    0]  # theta, rad between -pi and pi
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

        self.MAX_ACCEL_PER_CYCLE = 500/FPS # rotation/s/cycle

        self.current_goal = self.pos
        self.goal_reached = True
        self.goals_positions = [[TABLE_WIDTH/2, TABLE_HEIGHT/2, 0],
                                ]  # [x, y, theta]

        # self.max_wheel_speed = 1  # rad/s
        self.max_ang_speed = 1.2  # pi/2  # rad/s

        # juste un P, normal pour la position
        self.pid_pos_x = PID(1, 0, 0, 1/FPS)
        self.pid_pos_y = PID(1, 0, 0, 1/FPS)
        self.pid_pos_theta = PID(50, 0, 0, 1/FPS)
        self.delta_t = 1 / FPS  # Temps entre deux mises à jour


    def write_speeds(self, speeds: List) -> None:
        self.vitesse_roue0 = speeds[0]
        self.vitesse_roue1 = speeds[1]
        self.vitesse_roue2 = speeds[2]

    def goto(self, x, y, theta):
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

        # # PID
        self.vitesse_lineaire = [
            self.pid_pos_x.update(x - self.pos[0]),
            self.pid_pos_y.update(y - self.pos[1])
        ]

    def goto_next_goal(self):
        self.goal_reached = False
        self.has_finished_rotation = False
        if len(self.goals_positions)>0:
            self.goals_positions.pop(0)
        if len(self.goals_positions)>0:
            self.current_goal = self.goals_positions[0]
        
    def check_angle(self, angle1, angle2, error_max):
        error = abs(angle1 - angle2)
        if (abs(2*pi-error) < 0.01):
            error = 0
        return error < error_max

    def check_goal_reached(self):
        error_max_lin = 1
        error_max_ang = 0.01

        if (len(self.goals_positions))==0:
            return True

        self.current_goal = self.goals_positions[0]

        if abs(self.pos[0] - self.current_goal[0]) < error_max_lin and abs(self.pos[1] - self.current_goal[1]) < error_max_lin:
            if self.check_angle(self.pos[2], self.current_goal[2], error_max_ang):
                self.goto_next_goal()
                ic("GOAL REACHED")

                self.goal_reached = True

    def check_rotation_reached(self):
        error_max_ang = 0.01
        return self.check_angle(self.pos[2], self.current_goal[2], error_max_ang)

    def update_robot_position(self):
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
        self.pos[0] += self.vitesse_lineaire[0] * self.delta_t
        self.pos[1] += self.vitesse_lineaire[1] * self.delta_t
        self.pos[2] += self.vitesse_angulaire * self.delta_t

        self.robot_positions.append(self.pos.copy())

    def vitesses_to_roues(self):
        # On calcule la vitesse de chaque roue en mètres par seconde
        cmd_vitesse_roue0 = 0.5 * \
            self.vitesse_lineaire[0] - sqrt(3) / 2 * self.vitesse_lineaire[1] - \
            self.rayon_robot * self.vitesse_angulaire
        cmd_vitesse_roue1 = 0.5 * \
            self.vitesse_lineaire[0] + sqrt(3) / 2 * self.vitesse_lineaire[1] - \
            self.rayon_robot * self.vitesse_angulaire
        cmd_vitesse_roue2 = self.vitesse_lineaire[0] - \
            self.rayon_robot * self.vitesse_angulaire


        # limit the speeds
        # calc les accels
        accel_roue_0 = cmd_vitesse_roue0 - self.vitesse_roue0
        accel_roue_1 = cmd_vitesse_roue1 - self.vitesse_roue1
        accel_roue_2 = cmd_vitesse_roue2 - self.vitesse_roue2
        
        abs_accel_roue_0 = abs(accel_roue_0)
        abs_accel_roue_1 = abs(accel_roue_1)
        abs_accel_roue_2 = abs(accel_roue_2)

        
        abs_accel_roues = [abs_accel_roue_0, abs_accel_roue_1, abs_accel_roue_2]

        if abs_accel_roue_0 < self.MAX_ACCEL_PER_CYCLE and abs_accel_roue_1 < self.MAX_ACCEL_PER_CYCLE and abs_accel_roue_2 < self.MAX_ACCEL_PER_CYCLE:
            # acceleration requested is ok, no need to accelerate gradually.
            self.write_speeds([cmd_vitesse_roue0, cmd_vitesse_roue1, cmd_vitesse_roue2])
        else:
            speed_ratio = self.MAX_ACCEL_PER_CYCLE / max(abs_accel_roues)
            self.write_speeds([self.vitesse_roue0 + speed_ratio * accel_roue_0, 
                               self.vitesse_roue1 + speed_ratio * accel_roue_1, 
                               self.vitesse_roue2 + speed_ratio * accel_roue_2])






class Obstacle_static_model():
    # carré
    def __init__(self, center_x, center_y, width, height, offset) -> None:
        self.center_x = center_x
        self.center_y = center_y
        self.width = width
        self.height = height
        self.polygon = Polygon([Point2(center_x+width/2,center_y+height/2),
                               Point2(center_x+width/2,center_y-height/2),
                               Point2(center_x-width/2,center_y-height/2),
                               Point2(center_x-width/2,center_y+height/2)])
        self.expanded_obstacle_poly = math_bind.expand(self.polygon, offset)

