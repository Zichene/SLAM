import numpy as np
from icp import icp
import g2o


def to_se2(odom):
    """
    Get the SE2 (3x3 matrix) representation of an odometry (or similar format) reading.
    :param odom: 2D Odometry reading of the form [x, y, theta]
    :return: np.array 3x3 matrix corresponding to the SE2 form of the odom reading.
    """
    assert len(odom) == 3
    x, y, theta = odom[0], odom[1], odom[2]
    return np.array([[np.cos(theta), -np.sin(theta), x],
                     [np.sin(theta), np.cos(theta), y],
                     [0, 0, 1]])


def add_odom_constraint_edge(graph, ids, odoms, lasers, current_pose, ignore_small_change=False, robust_kernel=True):
    """
    Adds the edge corresponding to the constraint between two successive poses to
    the graph. This is the 'odometry' constraint edge. Also adds the second vertex
    corresponding to `odoms[1]`.

    *NOTE: Assumes that the vertex corresponding to 'id' has already been added to the graph.

    :param graph: The Graph object.
    :param ids: The IDs of the two successive readings.
    :param odoms: Array of two successive odometry readings.
    :param lasers: Array of two successive lasers readings.
    :param current_pose: Current pose of the robot from the main SLAM loop
    :param ignore_small_change: If true, ignores small changes.
    :param robust_kernel: If true, adds g2o.RobustKernelDCS() object to the edge.
    :return: True if adding was successful, False if a small change was ignored.
    """
    assert len(odoms) == len(lasers) == 2
    distance_small_change_threshold = 0.4  # in m
    angle_small_change_threshold = 0.2  # in rad

    # check that a 'significant change has happened'
    diff = odoms[1] - odoms[0]
    if ignore_small_change and (np.linalg.norm(diff[0:2]) < distance_small_change_threshold
                                and diff[2] < angle_small_change_threshold):
        return False, current_pose

    # scan match lasers using ICP
    laser_set_A = lasers[0]
    laser_set_B = lasers[1]
    initial_pose = to_se2(diff)

    with np.errstate(all='raise'):
        try:
            transformation, distances, num_iter, cov = icp(
                laser_set_B,
                laser_set_A,
                initial_pose,
                max_iterations=100,
                tolerance=0.00001,
            )
        except Exception as e:
            return False, current_pose
    # print(transformation)
    # print(distances)
    # we could probably remove this since cov is always going to be the identity 3x3
    information = np.linalg.inv(cov)

    # get the pose of second vertex via transformation and add to graph
    #pose = graph.get_pose(ids[0]).to_isometry().matrix()
    current_pose = np.matmul(current_pose, transformation)
    graph.add_vertex(ids[1], g2o.SE2(g2o.Isometry2d(current_pose)))

    # add edge to graph
    graph.add_edge(vertices=[ids[0], ids[1]],
                   measurement=g2o.SE2(g2o.Isometry2d(transformation)),
                   information=information,
                   robust_kernel=g2o.RobustKernelDCS() if robust_kernel else None
                   )
    return True, current_pose
