import numpy as np
import g2o
import matplotlib.pyplot as plt

class Graph():
    def __init__(self):
        self.optimizer = g2o.SparseOptimizer()
        solver = g2o.BlockSolverSE2(g2o.LinearSolverEigenSE2())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        self.optimizer.set_algorithm(solver)

    def optimize(self, max_iterations=40):
        """
        Use the g2o optimizer to optimize the graph.
        """
        self.optimizer.initialize_optimization()
        self.optimizer.optimize(max_iterations)

    def add_vertex(self, id, pose, fixed=False):
        """
        Add a vertex to the graph with given id and pose. The vertices
        should actually be added as `g2o.VertexSE2` objects. They should be
        added to the `self.optimizer` object using the `add_vertex` method from
        the optimizer class.

        :param id: ID of the given vertex
        :param pose: A g2o.SE2 object representing a robot pose.
        :param fixed: Whether the SE2 vertex is fixed, defaults to False
        """
        v_se2 = g2o.VertexSE2()
        v_se2.set_id(id)
        v_se2.set_estimate(pose)
        v_se2.set_fixed(fixed)
        self.optimizer.add_vertex(v_se2)

    def add_edge(self, vertices, measurement,
                 information=np.identity(3),
                 robust_kernel=None):
        """
        Adds an edge to the graph. Edges represent a 'constraint'. The edge
        should actually be added as `g2o.EdgeSE2` objects. They should be
        added to the `self.optimizer` object using the `add_edge` method from
        the optimizer class.

        :param vertices: The vertices that are connected to the edge. Could also be
        indices representing the id of vertices which are already in the graph.
        :param measurement: A SE2 object representing the measurement that defines the constraint.
        :type measurement: g2o.SE2
        :param information: Information matrix (np array: 3x3). Inverse of the covariance matrix. Defaults to np.identity(3).
        :type information: np.array
        :param robust_kernel: Should be a g2o.RobustKernelDCS() object, or None. Defaults to None.
        :type robust_kernel: g2o.RobustKernelDCS, optional
        """

        edge = g2o.EdgeSE2()
        for i, v in enumerate(vertices):
            if isinstance(v, int):
                v = self.optimizer.vertex(v)
            edge.set_vertex(i, v)

        # relative pose
        edge.set_measurement(measurement)
        edge.set_information(information)
        if robust_kernel is not None:
            edge.set_robust_kernel(robust_kernel)
        self.optimizer.add_edge(edge)

    def get_pose(self, id):
        return self.optimizer.vertex(id).estimate()

    def plot(self, num_vertex, registered_lasers, show_point_cloud=False):
        """
        Plot the computed trajectory and optionally the cloud points if given the registered_lasers data.

        :param num_vertex: The amount of vertices to plot for. This is typically the last vertex index (vertex_idx).
        :type num_vertex: int
        :param registered_lasers: A list of registered laser scans for each registered vertices.
        :type registered_lasers: list, optional
        :param show_point_cloud: Wether to plot the registered to the point clouds, defaults to False.
        :type show_point_cloud: bool, optional
        """
        if show_point_cloud:
            assert len(registered_lasers) == num_vertex

        traj = []
        point_cloud = []

        for idx in range(num_vertex):
            traj.append(self.get_pose(idx).to_vector()[:2])

            if show_point_cloud:
                x = self.get_pose(idx)
                r = x.to_isometry().R
                t = x.to_isometry().t
                filtered = registered_lasers[idx].T
                filtered = filtered[np.linalg.norm(filtered, axis=1) < 80]
                point_cloud.append((r @ filtered.T + t[:, np.newaxis]).T)

        # Trajectory
        traj = np.array(traj)
        plt.plot(traj[:, 0], traj[:, 1], '-g')

        # Point cloud
        if show_point_cloud:
            point_cloud = np.vstack(point_cloud)
            xyreso = 0.01 # Map resolution (m)
            point_cloud = (point_cloud / xyreso).astype('int')
            point_cloud = np.unique(point_cloud, axis=0)
            point_cloud = point_cloud * xyreso

            # Plot
            plt.plot(point_cloud[:, 0], point_cloud[:, 1], '.b', markersize=0.01)

        # Plot
        plt.show()