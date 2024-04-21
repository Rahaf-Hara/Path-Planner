import math
from .utils import is_occluded


class Node:
    """Represents a node in a graph with spatial coordinates and connectivity."""

    def __init__(self, x: int, y: int, idx: int):
        """Initializes a new instance of the Node class."""
        self.x = x
        self.y = y
        self.idx = idx
        self.neighbours = []
        self.neighbour_costs = []
        self.cost = float('inf')
        self.parent_node = None

    def distance_to(self, other: 'Node') -> float:
        """Calculate the Euclidean distance to another node."""
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    def is_connected(self, img, other_node: 'Node') -> bool:
        """Determine if there is a direct, unobstructed path to another node."""
        p1 = [self.x, self.y]
        p2 = [other_node.x, other_node.y]
        return not is_occluded(img, p1, p2)
