"""
This class implements a Graph for Graph-SLAM. The nodes of this
graph represent the robot poses, and the edges represent 'constraints'
between two different nodes.
"""


class Graph:
    class Node:
        def __init__(self, pose, is_landmark: bool = False):
            self.pose = pose
            self.neighbors = []
            self.is_landmark = is_landmark

    class Constraint:
        """
        This class implements a constraint between two nodes.
        Odometry constraint: x_{i+1} = x_i + D (D is the odom measurement of distance between
        x_{i+1} and x_i, FROM THE PERSPECTIVE OF x_i)
        Landmark constraint: x_j = m_k + L (L is measurement of distance between node x_j
        and landmark m_k, FROM THE PERSPECTIVE OF THE LANDMARK m_k)
        """

        def __init__(self, nodes, measurement):
            self.nodes = nodes
            self.measurement = measurement

    def __init__(self, root: Node):
        assert root is not None
        self.root = root
        self.nodes = []
        self.edges = []
        self.nodes.append(root)

    def add_node(self, pose):
        assert pose is not None
        self.nodes.append(Graph.Node(pose))

    def add_edge(self, nodes, measurement):
        assert nodes is not None
        self.edges.append(self.Constraint(nodes, measurement))
        nodes[0].neighbors.append(nodes[1])
        nodes[1].neighbors.append(nodes[0])