from math import *
import skgeom as sg 

def expand(poly: sg.Polygon, offset: float) -> sg.Polygon:
    """Expand a polygon by a given offset and return the expanded polygon"""
    if offset>0:
        skel = sg.skeleton.create_exterior_straight_skeleton(poly,offset)
    else:
        offset=-offset
        skel = sg.skeleton.create_interior_straight_skeleton(poly)
    expanded_poly = skel.offset_polygons(offset)[0]
    return expanded_poly

def eq_tuples(pointA: tuple, pointB: tuple) -> bool:
    """Check if two tuples of points are almost equals at the 2nd decimal"""
    return round(float(pointA[0]), 2) == round(float(pointB[0]), 2) and round(float(pointA[1]), 2) == round(float(pointB[1]), 2) 

def eq_points(pointA: sg.Point2, pointB: sg.Point2) -> bool:
    """Check if two points are almost equals at the 2nd decimal"""
    return round(float(pointA.x()), 2) == round(float(pointB.x()), 2) and round(float(pointA.y()), 2) == round(float(pointB.y()), 2) 

def eq_segs(segA: sg.Segment2, segB: sg.Segment2) -> bool:
    """Check if two segments are almost equals at the 2nd decimal"""
    return eq_points(segA[0], segB[0]) and eq_points(segA[1],segB[1])

def dist(A: sg.Point2,B: sg.Point2) -> float:
    """Compute the euclidian distance between two points"""
    return sqrt(pow(B[0]-A[0],2)+pow(B[1]-A[1],2))

def point_to_tuple(p: sg.Point2) -> tuple:
    """Transform a Point2 to a tuple of coordinates"""
    return (float(p.x()),float(p.y()))

def tuple_to_point(t: tuple) -> sg.Point2:
    """Transform a tuple of coordinates to a Point2"""
    return sg.Point2(t[0],t[1])