import g2o
import numpy as np
from graph import Graph

def run_visual_slam():
    # testing graph object
    graph = Graph()

    graph.add_vertex(
        id=0,
        pose=g2o.Isometry3d(np.eye(4)),
        fixed=True,
    )

    graph.add_vertex(
        id=1,
        pose=g2o.Isometry3d(np.eye(4)),
        fixed=True,
    )

    graph.add_edge(
        vertices=[0, 1],
        measurement=g2o.Isometry3d(np.eye(4)),
        robust_kernel=g2o.RobustKernelDCS(),
    )

    print(len(graph.optimizer.vertices().items()))
    print(len(graph.optimizer.edges()))
    # can get the matrix
    print(graph.get_pose(id=0).matrix())
    # can also get the translation vector
    print(graph.get_pose(id=0).t)
    # can also get the quaternion
    print(graph.get_pose(id=0).Quaternion())







if __name__ == "__main__":
    run_visual_slam()

