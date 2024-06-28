import g2o
import numpy as np

class Graph:
    """
    A Graph class template inspired from:
    https://github.com/goktug97/PyGraphSLAM/
    This class makes use of g2o's SparseOptimizer to optimize the graph.
    """
    def __init__(self):
        # have the optimizer from g2o as a member of the class
        self.optimizer = g2o.SparseOptimizer()
        solver = g2o.BlockSolverSE3(g2o.LinearSolverEigenSE3())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        self.optimizer.set_algorithm(solver)
        self.ids = []

    def optimize(self, max_iterations=20):
        """
        Use the g2o optimizer to optimize the graph.
        """
        self.optimizer.initialize_optimization()
        self.optimizer.optimize(max_iterations)

    def add_vertex(self, id, pose, fixed=False):
        """
        Add a vertex to the graph with given id and pose. The vertices
        should actually be added as `g2o.VertexSE3` objects. They should be
        added to the `self.optimizer` object using the `add_vertex` method from
        the optimizer class.
        
        :param id: ID of the given vertex (int)
        :param pose: An g2o.Isometry3d object representing a robot pose. 
        :param fixed: Whether the SE3 vertex is fixed.
        """
        assert isinstance(pose, g2o.Isometry3d)
        
        se3_vertex = g2o.VertexSE3()
        se3_vertex.set_estimate(pose)
        se3_vertex.set_id(id)
        se3_vertex.set_fixed(fixed)
        self.optimizer.add_vertex(se3_vertex)
        self.ids.append(id)

    def add_edge(self,
                 vertices,
                 measurement,
                 information=np.identity(6),
                 robust_kernel=None):
        """
        Adds an edge to the graph. Edges represent a 'constraint'. The edge
        should actually be added as `g2o.EdgeSE3` objects. They should be
        added to the `self.optimizer` object using the `add_edge` method from
        the optimizer class.
        
        :param vertices: The indices of the vertices that are connected to the edge.
        :param measurement: An g2o.Isometry3D object representing the measurement that defines the constraint.
        :param information: Information matrix (np array: 6x6). Inverse of the covariance matrix.
        :param robust_kernel: Should be a g2o.RobustKernelDCS() object, or None.
        """
        se3_edge = g2o.EdgeSE3()
        for i, v in enumerate(vertices):
            # if v is an integer, we are passing in indices, find the vertex from graph
            if isinstance(v, int):
                v = self.optimizer.vertex(v)
                assert v is not None
            se3_edge.set_vertex(i, v)
            se3_edge.set_measurement(measurement)
            se3_edge.set_information(information)
            if robust_kernel is not None:
                se3_edge.set_robust_kernel(robust_kernel)
            self.optimizer.add_edge(se3_edge)

    def get_pose(self, id) -> g2o.Isometry3d:
        """
        Get a pose from the graph from given ID.
        :param id: id of pose.
        :return: pose corresponding to the ID, it is an Isometry3D object.
        """
        assert self.optimizer.vertex(id) is not None
        return self.optimizer.vertex(id).estimate()