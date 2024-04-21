import rospy


class GraphSearch:
    def __init__(self, graph, start_xy=None, goal_xy=None):
        self.graph_ = graph
        self.heuristic_weight_ = rospy.get_param("~heuristic_weight")

        if start_xy and goal_xy:
            self.start_idx_ = self.graph_.get_closest_node(start_xy)
            self.goal_idx_ = self.graph_.get_closest_node(goal_xy)
            print(self.start_idx_)
            self.search(self.start_idx_, self.goal_idx_)
            self.path_ = self.generate_path(self.goal_idx_)
            self.graph_.visualize_path(self.path_)

    def search(self, start_idx, goal_idx):
        """Performs a search to find the shortest path from start to goal using cost."""
        # Initialize all nodes with infinite cost and no parents
        for node in self.graph_.nodes_:
            node.cost = float('inf')
            node.parent_node = None

        unvisited_set = [start_idx]
        visited_set = []

        # The start node has zero cost
        self.graph_.nodes_[start_idx].cost = 0

        # Loop until solution found or graph is disconnected
        while unvisited_set:

            # Find the node with the minimum cost in the unvisited set
            current_idx = self.get_minimum_cost_node(unvisited_set)
            current_node = self.graph_.nodes_[current_idx]

            # Move the node from unvisited to visited
            unvisited_set.remove(current_idx)
            visited_set.append(current_idx)

            # Termination criteria
            if current_idx == goal_idx:
                rospy.loginfo("Goal found!")
                return

            # Explore each neighbor of the current node
            for neighbor, cost in zip(current_node.neighbours, current_node.neighbour_costs):
                if neighbor.idx in visited_set:
                    continue

                # Compute the cost of this neighbour node
                new_cost = current_node.cost + cost + self.heuristic_weight_ * neighbor.distance_to(self.graph_.nodes_[goal_idx])

                # Check if neighbours is already in unvisited
                if neighbor.idx in unvisited_set:
                    if new_cost < neighbor.cost:
                        neighbor.cost = new_cost
                        neighbor.parent_node = current_node
                else:
                    unvisited_set.append(neighbor.idx)
                    neighbor.cost = new_cost
                    neighbor.parent_node = current_node

            # Visualise the current search status in RVIZ
            self.visualise_search(visited_set, unvisited_set, start_idx, goal_idx)

    def get_minimum_cost_node(self, unvisited_set):
        """Finds the node with the lowest cost in the unvisited set."""
        min_cost = float('inf')
        min_idx = None
        for idx in unvisited_set:
            if self.graph_.nodes_[idx].cost < min_cost:
                min_cost = self.graph_.nodes_[idx].cost
                min_idx = idx
        return min_idx

    def generate_path(self, goal_idx):
        """Generates a path from the start node to the goal node using parent pointers."""
        path = []
        current_node = self.graph_.nodes_[goal_idx]

        while current_node is not None:
            path.append(current_node)
            current_node = current_node.parent_node
        # Reverse the path since it was constructed from goal to start
        return path[::-1]

    def find_connected_nodes(self, start_idx):
        """Determines all nodes that are reachable from the specified start index."""
        # Set all parents and costs to zero
        for node in self.graph_.nodes_:
            node.cost = float('inf')
            node.parent_node = None

        # Setup sets.
        unvisited_set = [start_idx]
        visited_set = []
        self.graph_.nodes_[start_idx].cost = 0

        while unvisited_set:
            current_idx = self.get_minimum_cost_node(unvisited_set)
            current_node = self.graph_.nodes_[current_idx]

            unvisited_set.remove(current_idx)
            visited_set.append(current_idx)

            # Explore each neighbor
            for neighbor, cost in zip(current_node.neighbours, current_node.neighbour_costs):
                if neighbor.idx not in visited_set:
                    new_cost = current_node.cost + cost
                    if neighbor.idx in unvisited_set:
                        if new_cost < neighbor.cost:
                            neighbor.cost = new_cost
                            neighbor.parent_node = current_node
                    else:
                        unvisited_set.append(neighbor.idx)
                        neighbor.cost = new_cost
                        neighbor.parent_node = current_node
        return visited_set

    def visualise_search(self, visited_set, unvisited_set, start_idx, goal_idx):
        self.graph_.visualize_search(visited_set, unvisited_set, start_idx, goal_idx)
