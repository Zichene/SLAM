import g2o
import numpy as np
from matplotlib import pyplot as plt

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
    odom_plt = [odom[0:2] for odom in odoms]
    odom_plt = np.array(odom_plt)
    plt.plot(odom_plt[:, 0], odom_plt[:, 1], '-g')
    plt.show()
    graph = Graph()
    prev_id = 0
    add_odom_edge_index = 0
    detect_loop_index = 0
    current_pose_matrix = np.eye(3)
    initial_pose = g2o.SE2(g2o.Isometry2d(current_pose_matrix))
    graph.add_vertex(0, initial_pose, fixed=True)
    loop_closures_idx = []
    for data_id in range(1, 4000):
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
            print(f"Adding edge: index {data_id}.")
            # here check for loop closure every ten iterations
            if add_odom_edge_index > 10 and not add_odom_edge_index % 10:
                if detect_loop_closure(graph, data_id, lasers):
                    print(f"Loop closure detected at index {data_id}."
                          f" Corresponding id {graph.loop_detected_ids_corresponding[detect_loop_index]}."
                          f" Mean dist {graph.mean_dists[detect_loop_index]}.")
                    detect_loop_index += 1
                    loop_closures_idx.append(data_id)

                    graph.plot()
                    pass

            graph.optimize()
            current_pose_matrix = graph.get_pose(data_id).to_isometry().matrix()
            add_odom_edge_index += 1
        if data_id % 250 == 0:
            graph.plot()

    #graph.optimize()
    print(len(graph.optimizer.vertices().items()))
    print(len(graph.optimizer.edges()))
    graph.optimize()
    graph.plot()


if __name__ == "__main__":
    run_slam()
