import g2o

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
    initial_pose = g2o.SE2(g2o.Isometry2d(to_se2([0, 0, 0])))
    graph.add_vertex(0, initial_pose)
    for data_id in range(1, len(odoms)-1):
        successive_odoms = [odoms[prev_id], odoms[data_id]]
        successive_lasers = [lasers[prev_id], lasers[data_id]]
        if add_odom_constraint_edge(
            graph=graph,
            ids=[prev_id, data_id],
            odoms=successive_odoms,
            lasers=successive_lasers,
            ignore_small_change=True,
        ):
            prev_id = data_id
            # here check for loop closure every ten iterations
            if add_odom_edge_index % 10 == 0:
                if detect_loop_closure(graph, data_id, lasers):
                    print(f"Loop closure detected at index {data_id}. Optimizing graph ... ")
                    graph.optimize()
                    print("Graph optimized.")
            add_odom_edge_index += 1

    print(len(graph.optimizer.vertices().items()))
    print(len(graph.optimizer.edges()))


if __name__ == "__main__":
    run_slam()
