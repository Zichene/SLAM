"""
This class implements a Graph for Graph-SLAM. The nodes of this
graph represent the robot poses, and the edges represent 'constraints'
between two different nodes.
"""
import array


class Graph:

    class Node:
        def __init__(self, pose: [float, float]):
            self.pose = pose
            self.neighbors = []

    class Constraint:
        """
        This class implements a constraint between two nodes.
        """
        def __init__(self, nodes):
            self.nodes = nodes

    def __init__(self, root: Node):
        assert root is not None
        self.root = root
        self.nodes = []
        self.edges = []
        self.nodes.append(root)

    def add_node(self, pose: [float, float]):
        assert pose is not None
        self.nodes.append(Graph.Node(pose))



