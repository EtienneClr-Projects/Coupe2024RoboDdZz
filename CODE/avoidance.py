# lib pour l'evitement d'obstacle grace a la methode du graphe de visibilité

from math_bind import *

from dijkstar import Graph, find_path
from skgeom import *
from icecream import ic


def create_graph(start: sg.Point2, goal: sg.Point2, expanded_obstacle_poly: sg.Polygon):
    """Create the graph of navigation from is point to the goal

    Args:
        start (sg.Point2): The start point of the robot (robot pos)
        goal (sg.Point2): The goal point of the robot
        expanded_obstacle_poly (sg.Polygon): The polygon of the obstacle (expanded)
    """

    ic("CREATING GRAPH\n\n")
    graph = Graph()

    # create a dictionnary associating each point with an index
    dico_all_points = {}
    dico_all_points[len(dico_all_points)] = (start.x(),start.y())
    dico_all_points[len(dico_all_points)] = (goal.x(),goal.y())
    for point in list(expanded_obstacle_poly.vertices):
        dico_all_points[len(dico_all_points)] = point_to_tuple(point)
    

# GENERATING COMBINATIONS
    # generate each combination between start/goal to every other points to test visibility
    all_combinations = []
    for key in dico_all_points.keys():
        if key != 0 and key != 1: # start and goal
            all_combinations.append((key, 0))
            all_combinations.append((key, 1))

    # Add the segment between start and goal as a combination to test
    all_combinations.append((0,1)) 

# ADDING EDGES TO THE GRAPH
    # generate the edges of the polygon that should not be crossed
    # Todo TROUVER UNE AUTRE METHODE
    poly_edges = []
    for i in range(len(dico_all_points)-2):
        if i==len(dico_all_points)-2-1:
            poly_edges.append((i+2, 2))
        else:
            poly_edges.append((i+2, i+2+1))

    # Add the edges of the polygon to the graph as they are admissible by nature
    for seg in poly_edges:
        d = dist(dico_all_points[seg[0]],dico_all_points[seg[1]])
        graph.add_edge(seg[0],seg[1],d)
        graph.add_edge(seg[1],seg[0],d)

    # check for each segment/combination of two points if they cross the obstacle
    # if not we add them as edges to the graph
    for comb in all_combinations:
        a, b = comb
        pointA = dico_all_points[a]
        pointB = dico_all_points[b]
        segment = Segment2(tuple_to_point(pointA),tuple_to_point(pointB))

        # Check if there is an intersection with one obstacle
        no_inter = True
        for edge in expanded_obstacle_poly.edges:
            inter = intersection(edge, segment)
            if inter == None:
                continue
            
            inter = point_to_tuple(inter)
            edgeA = point_to_tuple(edge[0])
            edgeB = point_to_tuple(edge[1])
            
            # if segments have a common point, having an intersection is OK
            if not eq_tuples(pointA, inter) and not eq_tuples(inter, pointB) and not eq_tuples(inter, edgeA) and not eq_tuples(inter,edgeB):
                no_inter = False
                break

        if no_inter:
            d = dist(dico_all_points[a],dico_all_points[b])
            graph.add_edge(a,b,d)
            graph.add_edge(b,a,d)
            # ic(
            #     str(pointA[0])+","+str(pointA[1]),
            #     str(pointB[0])+","+str(pointB[1])
            # )
            # print()


    ic("CREATED GRAPH\n\n")

    return graph,dico_all_points



def find_avoidance_path(graph, start, end):
    """Find a path between the start and the end with the given graph if possible

    Args:
        graph (Graph): The graph of visibility
        start (sg.Point2): The start point
        end (sg.Point2): The goal point

    Returns:
        list[int]: The list of the indices of the points of the path
    """
    return find_path(graph, start, end)
