# lib pour l'evitement d'obstacle grace a la methode du graphe de visibilité

from dijkstar import Graph, find_path
from skgeom import *
from skgeom.draw import draw


# recuperation des points des objets de la table
def create_graph(robot, obstacle, table):
    # robot = [x, y, r]
    # obstacle = [x_center, y_center, width, height]
    # table = [x, y, width, height]

    # ajout de l'obstacle
    o_center_x = obstacle[0]
    o_center_y = obstacle[1]
    o_width = obstacle[2]
    o_height = obstacle[3]
    # create a polygon
    obstacle_poly = Polygon((o_center_x, o_center_y),
                            (o_center_x + o_width, o_center_y),
                            (o_center_x + o_width, o_center_y + o_height),
                            (o_center_x, o_center_y + o_height))

    # offset de l'obstacle
    offset = 10
    expanded_obstacle_poly = offset_polygon(obstacle_poly, offset)

    # ajout de la table
    t_x = table[0]
    t_y = table[1]
    t_width = table[2]
    t_height = table[3]
    # create a polygon
    table_poly = Polygon((t_x, t_y),
                         (t_x + t_width, t_y),
                         (t_x + t_width, t_y + t_height),
                         (t_x, t_y + t_height))

    # creation du graphe
    graph = Graph()

    # on cree le graphe de visibilité
    # on ajoute les points du robot
    robot_x = robot[0]
    robot_y = robot[1]
    robot_r = robot[2]
    graph.add_node("robot")

    # pour chaque combinaison de points de l'obstacle avec le point du robot on ajoute une arete si il n'y a pas d'obstacle entre les deux points
    for i in range(4):
        graph.add_node("obstacle" + str(i))
        for j in range(4):
            if i != j:
                # on verifie si il n'y a pas d'obstacle entre les deux points
                # on cree une droite entre les deux points
                line = Line(
                    expanded_obstacle_poly.vertices[i], expanded_obstacle_poly.vertices[j])
                # on verifie si la droite ne coupe pas l'obstacle
                if not line.intersection(expanded_obstacle_poly):
                    # on verifie si la droite ne coupe pas la table
                    if not line.intersection(table_poly):
                        graph.add_edge("obstacle" + str(i),
                                       "obstacle" + str(j), 1)
                        graph.add_edge("obstacle" + str(j),
                                       "obstacle" + str(i), 1)
                        # # on ajoute les points de l'obstacle
                        # graph.add_node("obstacle" + str(j), expanded_obstacle_poly.vertices[j])
    return graph

# recherche du chemin le plus court


def find_avoidance_path(graph, start, end):
    return find_path(graph, start, end)


if __name__ == "__main__":
    poly = Polygon([Point2(0, 0), Point2(0, 3), Point2(3, 3)])
    draw(poly)

