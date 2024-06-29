import numpy as np
import g2o
import matplotlib.pyplot as plt

class PoseGraphOptimization(g2o.SparseOptimizer):
    def __init__(self):
        super().__init__()
        solver = g2o.BlockSolverSE2(g2o.LinearSolverEigenSE2())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        super().set_algorithm(solver)

    def optimize(self, max_iterations=20):
        super().initialize_optimization()
        super().optimize(max_iterations)

    def add_vertex(self, id, pose, fixed=False):
        v_se2 = g2o.VertexSE2()
        v_se2.set_id(id)
        v_se2.set_estimate(pose)
        v_se2.set_fixed(fixed)
        super().add_vertex(v_se2)

    def add_edge(self, vertices, measurement,
                 information=np.identity(3),
                 robust_kernel=None):

        edge = g2o.EdgeSE2()
        for i, v in enumerate(vertices):
            if isinstance(v, int):
                v = self.vertex(v)
            edge.set_vertex(i, v)

        # relative pose
        edge.set_measurement(measurement)
        edge.set_information(information)
        if robust_kernel is not None:
            edge.set_robust_kernel(robust_kernel)
        super().add_edge(edge)

    def get_pose(self, id):
        return self.vertex(id).estimate()

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