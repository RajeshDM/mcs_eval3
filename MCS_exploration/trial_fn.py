import math
from icecream import ic
import operator
from functools import reduce
import random

chest_pose = ([{'x': 4.302852, 'y': 0.144211963, 'z': -1.52183354},
             {'x': 4.01685238, 'y': 0.461999953, 'z': -1.73721552},
             {'x': 4.588852, 'y': 0.461999953, 'z': -1.73721552},
             {'x': 4.588852, 'y': 0.461999953, 'z': -1.31921554},
             {'x': 4.01685238, 'y': 0.461999953, 'z': -1.31921554}],
              180  )

def get_2d_distance(point_1, point_2):
    if isinstance(point_1,dict):
        return math.sqrt(math.pow(abs(point_1['x'] - point_2['x'] ),2) + math.pow(abs(point_1['z']-point_2['z']),2))
    else :
        return math.sqrt(math.pow(abs(point_1[0] - point_2[0] ),2) + math.pow(abs(point_1[1]-point_2[1]),2))


def get_polar_sorted_coords(outermost_pts):
    center = tuple(map(operator.truediv, reduce(lambda x, y: map(operator.add, x, y), outermost_pts), [len(outermost_pts)] * 2))
    sorted_coords  = sorted(outermost_pts, key=lambda coord: (-135 - math.degrees(math.atan2(*tuple(map(operator.sub, coord, center))[::-1]))) % 360)
    return sorted_coords 


def get_line_angle(pt_1, pt_2):
    nr = pt_2[1] - pt_1[1]
    dr = pt_2[0] - pt_1[0]
    
    if nr == 0 :
        if dr >= 0 :
            return 0
        else :
            return 180
    if dr == 0 :
        if nr >= 0 :
            return 90
        else :
            return 270

    slope = nr/dr
    return math.degrees(math.atan(slope)) 

def generate_point_from_centre_and_face(centre,pt_1,pt_2):
    r_min = (get_2d_distance(centre,pt_1))
    r_max = r_min * 2
    #ic(get_2d_distance(centre,pt_2))
    
    angle_1 = get_line_angle(centre,pt_1)/2
    angle_2 = get_line_angle(centre,pt_2)/2

    min_angle = min(angle_1,angle_2)
    max_angle = max(angle_2,angle_1)

    r = random.uniform(r_min*1.4, r_max)
    ic(r)
    ic(angle_1,angle_2)
    #theta = random.uniform() * 2 * PI
    theta = random.uniform(min_angle,max_angle) + 270
    ic (theta)
    ic (centre)
    x = centre[0] + r * math.cos(math.radians(theta))
    y = centre[1] + r * math.sin(math.radians(theta))
    ic(x,y)
    return (x,y)

def fn (object_data, object_pose):
    outer_points = []
    for i,point in enumerate(object_pose[0]):
        pt = (point['x'],point['z'])
        if i == 0 :
            centre = pt
        else :
            outer_points.append(pt)
    #pt_1 = (pt_1['x'], )
    #polar_sorted_coords = get_polar_sorted_coords([pt_1,pt_2,pt_3,pt_4])
    polar_sorted_coords = get_polar_sorted_coords(outer_points)
    polar_sorted_coords.reverse()
    lines_angle = []

    for i in range(len(polar_sorted_coords)):
        k = (i+ 1) % len(polar_sorted_coords)
        #ic (i,k)
        lines_angle.append((get_line_angle(polar_sorted_coords[i], polar_sorted_coords[k]),(i,k)))
        #ic (angle)

    obj_rotation = object_pose[1]    

    if obj_rotation <= 180 :
        real_angle = 180 - obj_rotation
    else :
        real_angle = 540 - obj_rotation

    #required_real_angle 
    ic (lines_angle) 
    for i,line in enumerate(lines_angle) :
        if line[0] == real_angle :
            face_location = (i + 1) % len(lines_angle) 
            break 

    ic(lines_angle[face_location][1] )
    ic(type(polar_sorted_coords))
    pt_1= polar_sorted_coords[lines_angle[face_location][1][0]]
    pt_2= polar_sorted_coords[lines_angle[face_location][1][1]]

    return generate_point_from_centre_and_face (centre, pt_1 , pt_2 )
        
random_point = fn(None, chest_pose)

