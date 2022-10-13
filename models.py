class Tree:
    def __init__(self, nodes, edges, weight = None):
        self.nodes = nodes
        self.edges = edges
        self.weight = weight

class Node:
    id_gen = 0

    def __init__(self, point):
        self.id = Node.id_gen
        Node.id_gen += 1

        self.point = point
        self.parent = None
        self.children = list()
        self.edges = list()
        self.seen = False

    def add_child(self, child, edge):
        self.children.append(child)
        self.edges.append(edge)


class Edge:
    def __init__(self, node_a, node_b):
        self.node_a = node_a
        self.node_b = node_b
        self.node_a.add_child(node_b, self)
