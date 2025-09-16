import numpy as np

def line_traversal(p0, p1, times, xlimit, ylimit, zlimit):
    '''
    
    Discritize a line
    
    Args:
        p0: start point
        p1: end point
        times: size of buffer zone
        x/y/zlimit: boundary of the grids
    
    return:
        results: discritized line
        neighbors: discritized line and its buffer zone
    
    '''
    # each element in p0 and p1 should be an integer
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]

    stepX = np.sign(dx)
    stepY = np.sign(dy)
    stepZ = np.sign(dz)

    tDeltaX = np.inf if dx == 0 else stepX / dx
    tDeltaY = np.inf if dy == 0 else stepY / dy
    tDeltaZ = np.inf if dz == 0 else stepZ / dz

    tMaxX = np.inf if dx == 0 else 0.5 * stepX / dx
    tMaxY = np.inf if dy == 0 else 0.5 * stepY / dy
    tMaxZ = np.inf if dz == 0 else 0.5 * stepZ / dz

    # make sure path node is a list
    current = list(p0)
    last = list(p1)

    results = []
    neighbors = []
    for i in range(max(0, current[0] - times), min(xlimit, current[0] + times + 1)):
        neighbors.append(list([i, current[1], current[2]]))
    for j in range(max(0, current[1] - times), min(ylimit, current[1] + times + 1)):
        neighbors.append(list([current[0], j, current[2]]))
    for k in range(max(0, current[2] - times), min(zlimit, current[2] + times + 1)):
        neighbors.append(list([current[0], current[1], k]))
    results.append(list(current))

    while current[0] != last[0] or current[1] != last[1] or current[2] != last[2]:
        if tMaxX < tMaxY:
            if tMaxX < tMaxZ:
                current[0] += stepX
                tMaxX += tDeltaX
                if current[2] >= 0:
                    for j in range(max(0, current[1] - times), min(ylimit, current[1] + times + 1)):
                        neighbors.append(list([current[0], j, current[2]]))
                    if 0 <= current[0] + stepX * times < xlimit:
                        neighbors.append(list([current[0] + stepX * times, current[1], current[2]]))
                for k in range(max(0, current[2] - times), min(zlimit, current[2] + times + 1)):
                    neighbors.append(list([current[0], current[1], k]))
            else:
                current[2] += stepZ
                tMaxZ += tDeltaZ
                if current[2] >= 0:
                    for i in range(max(0, current[0] - times), min(xlimit, current[0] + times + 1)):
                        neighbors.append(list([i, current[1], current[2]]))
                    for j in range(max(0, current[1] - times), min(ylimit, current[1] + times + 1)):
                        neighbors.append(list([current[0], j, current[2]]))
                if 0 <= current[2] + stepZ * times < zlimit:
                    neighbors.append(list([current[0], current[1], current[2] + stepZ * times]))
        else:
            if tMaxY < tMaxZ:
                current[1] += stepY
                tMaxY += tDeltaY
                if current[2] >= 0:
                    for i in range(max(0, current[0] - times), min(xlimit, current[0] + times + 1)):
                        neighbors.append(list([i, current[1], current[2]]))
                    if 0 <= current[1] + stepY * times < ylimit:
                        neighbors.append(list([current[0], current[1] + stepY * times, current[2]]))
                for k in range(max(0, current[2] - times), min(zlimit, current[2] + times + 1)):
                    neighbors.append(list([current[0], current[1], k]))
            else:
                current[2] += stepZ
                tMaxZ += tDeltaZ
                if current[2] >= 0:
                    for i in range(max(0, current[0] - times), min(xlimit, current[0] + times + 1)):
                        neighbors.append(list([i, current[1], current[2]]))
                    for j in range(max(0, current[1] - times), min(ylimit, current[1] + times + 1)):
                        neighbors.append(list([current[0], j, current[2]]))
                if 0 <= current[2] + stepZ * times < zlimit:
                    neighbors.append(list([current[0], current[1], current[2] + stepZ * times]))

        results.append(list(current))

    return results, neighbors

def find_best_normal_port(apt_i, apt_j):
    '''
    
    Find the best port of the airports for each route
    
    Args:
        apt_i: departure apt
        apt_j: arrival apt
        
    Returns:
        p_dep: port for departure
        p_arr: port for arrival
    
    '''
    od_track_angle = np.arctan2(apt_i.center[0] - apt_j.center[0], apt_i.center[1] - apt_j.center[1]) * 180 / np.pi + 180

    dep_usage = [p.occupied for p in apt_i.ports]
    # sort by port usage
    p_dep = apt_i.ports[np.argmin(dep_usage)]
    # sort by port angle
    if type(p_dep) == list and len(p_dep) > 1:
        dep_angles = [p.angle for p in p_dep]
        p_dep = p_dep[np.argmin(dep_angles - od_track_angle)]

    arr_usage = [p.occupied for p in apt_j.ports]
    # sort by port usage
    p_arr = apt_j.ports[np.argmin(arr_usage)]
    # sort by port angle
    if type(p_arr) == list and len(p_arr) > 1:
        arr_angles = [p.angle for p in p_arr]
        p_arr = p_arr[np.argmin(arr_angles - od_track_angle)]

    return p_dep, p_arr

