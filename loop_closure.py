from scipy.spatial import cKDTree

def detect_loop_closure(graph, id):
    """
    Detects a loop closure by comparing the laser scans at the current id to
    other positions which have a 'similar' odom reading. Implemented using
    a KDTree.
    :param graph: a Graph object
    :param id: ID of vertex which we want to compare
    :return: SE2 object corresponding to the constraint caused by loop detection,
    and the information matrix. Returns None if no detection occurs.
    """
    graph.get_pose(0).to_vector()
    # get the (x,y) for each pose in the graph
    poses = [graph.get_pose(idx).to_vector[0:2] for idx in graph.ids]

    pass
