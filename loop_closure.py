import g2o
import numpy as np
from scipy.spatial import cKDTree

from icp import icp


def detect_loop_closure(graph, id, lasers, robust_kernel=True):
    """
    Detects a loop closure by comparing the laser scans at the current id to
    other positions which have a 'similar' odom reading. Only compares laser scans
    between poses which are close (using kd-tree). Also adds the edge to the graph.

    *NOTE: The vertex corresponding to 'id' MUST already be in the graph.

    :param graph: a Graph object.
    :param id: ID of vertex which we want to compare.
    :param lasers: all the laser scans from dataset.
    :param robust_kernel: If true, adds g2o.RobustKernelDCS() object to the edge.
    :return: True if a loop closure is detected, False otherwise.
    """
    tolerance = 0.15
    kd_closest_threshold = 4.25
    graph.get_pose(0).to_vector()
    # get the (x,y) for each pose in the graph
    poses = [graph.get_pose(idx).to_vector()[0:2] for idx in graph.ids]
    kdtree = cKDTree(poses)
    current_pose = graph.get_pose(id).to_vector()[0:2]
    # get poses which are closest to current_pose using kd
    closest_indices = kdtree.query_ball_point(current_pose, r=kd_closest_threshold)
    # perform laser scan matching on closest indices
    for index in closest_indices:
        laser_set_A = lasers[graph.ids[index]]
        laser_set_B = lasers[id]
        transformation, distances, num_iter, cov = icp(
            laser_set_A,
            laser_set_B,
            np.eye(3),
            max_iterations=80,
            tolerance=0.0001
        )
        # if we have a good enough match, loop closure detected
        mean_dist = np.mean(distances)
        if np.mean(distances) < tolerance and graph.ids[index] != id:
            information = np.linalg.inv(cov)
            # add new edge corresponding to loop closure constraint to graph
            graph.add_edge(
                vertices=[graph.ids[index], id],
                measurement=g2o.SE2(g2o.Isometry2d(transformation)),
                information=information,
                robust_kernel=g2o.RobustKernelDCS() if robust_kernel else None
            )
            return True
    return False
