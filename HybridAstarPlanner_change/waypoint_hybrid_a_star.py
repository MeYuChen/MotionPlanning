def waypoint_hybrid_astar(qstart, qgoal, topological_graph, segment_graph, grid_map):
    exit_path, detachment_pose = find_exit_path(qstart, segment_graph)
    entry_path, attach_pose = find_entry_path(qgoal, segment_graph)
    start_area = goal_area = None

    for area in topological_graph:
        if even_odd_rule(qstart, area):
            start_area = area
        if even_odd_rule(qgoal, area):
            goal_area = area

    if start_area != goal_area:
        areas_sequence = dijkstra_algorithm(topological_graph, start_area, goal_area)
        waypoints = find_waypoints_sequence(areas_sequence)
        hybrid_astar_paths = []
        num_waypoints = len(waypoints)

        for i in range(num_waypoints):
            if i == 0:
                hybrid_astar_path0 = hybrid_astar(detachment_pose, waypoints[i], grid_map)
                hybrid_astar_paths.append(hybrid_astar_path0)
            elif i == num_waypoints - 1:
                hybrid_astar_pathi = hybrid_astar(waypoints[i], attach_pose, grid_map)
            else:
                hybrid_astar_pathi = hybrid_astar(waypoints[i], waypoints[i+1], grid_map)
            hybrid_astar_paths.append(hybrid_astar_pathi)
    else:
        hybrid_astar_paths = [hybrid_astar(detachment_pose, attach_pose, grid_map)]

    final_path = exit_path + hybrid_astar_paths + entry_path
    return final_path
def even_odd_rule(point, polygon):
    x, y = point
    inside = False

    n = len(polygon)
    p1x, p1y = polygon[0]
    for i in range(n+1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xints = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                    if p1x == p2x or x <= xints:
                        inside = not inside
        p1x, p1y = p2x, p2y

    return inside
