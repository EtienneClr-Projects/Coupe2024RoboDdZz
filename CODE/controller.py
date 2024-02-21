from kinematic_models import Robot_Kinematic_Model, Obstacle_static_model, Table_static_model
import avoidance

from icecream import ic
from dijkstar import Graph
from math import pi
import skgeom as sg

WIDTH, HEIGHT = 900, 600  # window
TABLE_WIDTH, TABLE_HEIGHT = 300, 200  # Table size in cm
FPS = 50

robot = Robot_Kinematic_Model(TABLE_WIDTH=TABLE_WIDTH, TABLE_HEIGHT=TABLE_HEIGHT,FPS=FPS)
offset = 10
obstacle = Obstacle_static_model(center_x=100, center_y= 100, width= 10, height= 10,offset=offset)
table = Table_static_model(TABLE_WIDTH, TABLE_HEIGHT, offset=offset)

def check_goal_reached():
    error_max_lin = 1
    error_max_ang = 0.01

    if (len(robot.goals_positions))==0:
        return True

    robot.current_goal = robot.goals_positions[0]

    if abs(robot.pos[0] - robot.current_goal[0]) < error_max_lin and abs(robot.pos[1] - robot.current_goal[1]) < error_max_lin:
        if robot.check_angle(robot.pos[2], robot.current_goal[2], error_max_ang):
            robot.goto_next_goal()
            ic("GOAL REACHED")

            robot.goal_reached = True

def goto(x, y, theta):
    # if it's shorter to turn in the other direction, we do it
    # TODO not always working
    error_theta = theta - robot.pos[2]
    if error_theta > pi:
        error_theta -= 2*pi
    elif error_theta < -pi:
        error_theta += 2*pi
    robot.angular_speed = robot.pid_pos_theta.update(error_theta)

    # # PID
    robot.linear_speed = [
        robot.pid_pos_x.update(x - robot.pos[0]),
        robot.pid_pos_y.update(y - robot.pos[1])
    ]


def goto_next_goal():
    # called when a goal is reached to set the new current goal
    robot.goal_reached = False
    robot.has_finished_rotation = False
    if len(robot.goals_positions)>0:
        robot.goals_positions.pop(0)
    if len(robot.goals_positions)>0:
        robot.current_goal = robot.goals_positions[0]
    

def check_angle(angle1, angle2, error_max):
    # check that the angle error is less than error_max
    error = abs(angle1 - angle2)
    if (abs(2*pi-error) < 0.01):
        error = 0
    return error < error_max



def recompute_path(obstacle, table, goal_pos=None):
    if goal_pos is None:
        if len(robot.goals_positions) > 0:
            theta = robot.goals_positions[-1][2] # use the theta of the goal for each point
            goal = sg.Point2(robot.goals_positions[-1][0],robot.goals_positions[-1][1])
        else:
            return
    else:
        theta = goal_pos[2]
        goal = sg.Point2(goal_pos[0],goal_pos[1])

    start = sg.Point2(robot.pos[0],robot.pos[1])
    robot.graph, robot.dico_all_points = avoidance.create_graph(start, goal, obstacle.expanded_obstacle_poly, table.expanded_poly)
    path = avoidance.find_avoidance_path(robot.graph, 0, 1)
    if path is not None:
        robot.path_nodes = path.nodes # mais en soit renvoie aussi le coût
        goals = []

        for p in robot.path_nodes[1:]: # we don't add the start point
            goals.append([float(robot.dico_all_points[p][0]),float(robot.dico_all_points[p][1]), theta])
        robot.goals_positions = goals










while True:
    goto(robot.current_goal[0],
         robot.current_goal[1],
         robot.current_goal[2])
            
    robot.recompute_path(obstacle, table)
    robot.check_goal_reached()
    robot.update_robot_position()

    """ENVOI MESSAGE ROS CMD_VEL"""
    """LIRE POSE GOAL, POLY ROBOT ADVERSE, ODOM"""