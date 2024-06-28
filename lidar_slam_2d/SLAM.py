import g2o
import numpy as np

from parse import get_dataset
from graph import Graph
from odom_constraint import add_odom_constraint_edge, to_se2
from loop_closure import detect_loop_closure


def run_slam():
    odoms, lasers = get_dataset()
    # odoms: np array of the form:
    # [[x_1,y_1,theta_1], [x_2,y_2,theta_2], ... ]
    # lasers: np array of the form:
    # [A, B, ...] where A,B,.. are each np.arrays of size (180, 2)
    assert len(odoms) == len(lasers)

    graph = Graph()
    prev_id = 0
    add_odom_edge_index = 0
    current_pose_matrix = to_se2([0, 0, 0])
    initial_pose = g2o.SE2(g2o.Isometry2d(current_pose_matrix))
    graph.add_vertex(0, initial_pose, fixed=True)
    for data_id in range(1, len(odoms)-1):
        successive_odoms = [odoms[prev_id], odoms[data_id]]
        successive_lasers = [lasers[prev_id], lasers[data_id]]
        success, pose_matrix = add_odom_constraint_edge(
            graph=graph,
            ids=[prev_id, data_id],
            odoms=successive_odoms,
            current_pose=current_pose_matrix,
            lasers=successive_lasers,
            ignore_small_change=True,
        )
        current_pose_matrix = pose_matrix
        if success:
            prev_id = data_id
            # here check for loop closure every ten iterations
            if add_odom_edge_index > 10 and not add_odom_edge_index % 10:
                if detect_loop_closure(graph, data_id, lasers):
                    print(f"Loop closure detected at index {data_id}.")
                    graph.optimize()
                    pass
            #graph.optimize()
            current_pose_matrix = graph.get_pose(data_id).to_isometry().matrix()
            add_odom_edge_index += 1

    #graph.optimize()
    print(len(graph.optimizer.vertices().items()))
    print(len(graph.optimizer.edges()))
    graph.plot()


if __name__ == "__main__":
    run_slam()
