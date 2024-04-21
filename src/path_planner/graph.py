import random
import rospy
import numpy as np
import matplotlib.cm as cm
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from .node import Node
from .graph_search import GraphSearch


class Graph:
    def __init__(self, map):
        self.map_ = map
        self.nodes_ = []
        self.groups_ = None
        self.load_ros_parameters()
        self.setup_publishers()
        self.initialize_markers()
        self.initialize_graph()

    def load_ros_parameters(self):
        """Load parameters from ROS parameter server."""
        self.grid_step_size_ = rospy.get_param("~grid_step_size")
        self.prm_num_nodes_ = rospy.get_param("~prm_num_nodes")
        self.use_prm = rospy.get_param("~use_prm")
        self.distance_threshold = rospy.get_param("~prm_max_edge_length")
        self.show_conn = rospy.get_param("~show_connectivity")

    def setup_publishers(self):
        """Setup ROS publishers for paths and visualization markers."""
        self.path_pub = rospy.Publisher('/path_planner/plan', Path, queue_size=10)
        self.path_smooth_pub = rospy.Publisher('/path_planner/plan_smooth', Path, queue_size=10)
        self.marker_pub_ = rospy.Publisher('marker', Marker, queue_size=10)

    def initialize_markers(self):
        """Initialize all ROS markers used for visualization."""
        self.marker_nodes_ = self.create_marker(Marker.POINTS, "nodes", 0.03, (1.0, 0.2, 0.2))
        self.marker_start_ = self.create_marker(Marker.POINTS, "start", 0.08, (1.0, 1.0, 0.2))
        self.marker_visited_ = self.create_marker(Marker.POINTS, "visited", 0.05, (0.2, 0.2, 1.0))
        self.marker_unvisited_ = self.create_marker(Marker.POINTS, "unvisited", 0.06, (0.3, 1.0, 0.3))
        self.marker_edges_ = self.create_marker(Marker.LINE_LIST, "edges", 0.008, (1.0, 1.0, 0.4))

    def create_marker(self, marker_type, namespace, scale, color):
        """Helper function to create a marker."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = namespace
        marker.id = 0
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = color
        return marker

    def initialize_graph(self):
        """Initialize the graph structure based on ROS parameters and visualize."""
        if self.use_prm:
            self.create_prm()
        else:
            self.create_grid()

        self.find_connected_groups()
        self.visualize_graph()

    def create_grid(self):
        """Create nodes in a grid pattern and connect nearby nodes that are not separated by an obstacle."""
        idx = 0
        for x in range(self.map_.min_x_, self.map_.max_x_ - 1, self.grid_step_size_):
            for y in range(self.map_.min_y_, self.map_.max_y_ - 1, self.grid_step_size_):
                if rospy.is_shutdown():
                    return

                if not self.map_.is_occupied(x, y):
                    self.nodes_.append(Node(x, y, idx))
                    idx += 1
        distance_threshold = self.grid_step_size_ * 1.01
        # Use a slightly higher distance than grid_step_size to connect strictly adjacent nodes
        self.create_edges(distance_threshold)

    def create_edges(self, max_distance):
        """Connect nodes based on their proximity and lack of obstacles."""
        count = 0
        for node_i in self.nodes_:
            count += 1
            print(f"{count} of {len(self.nodes_)}")
            if rospy.is_shutdown():
                return

            for node_j in self.nodes_:
                if node_i != node_j:
                    distance = node_i.distance_to(node_j)
                    if distance < max_distance:
                        if node_i.is_connected(self.map_.image_, node_j):
                            node_i.neighbours.append(node_j)
                            node_i.neighbour_costs.append(distance)

    def create_prm(self):
        """Generate a Probabilistic Roadmap (PRM) by placing nodes randomly and connecting them based on a maximum distance threshold."""
        idx = 0
        num_nodes = self.prm_num_nodes_

        while idx < num_nodes:
            if rospy.is_shutdown():
                return

            x = int(random.uniform(self.map_.min_x_, self.map_.max_x_))
            y = int(random.uniform(self.map_.min_y_, self.map_.max_y_))

            if not self.map_.is_occupied(x, y):
                self.nodes_.append(Node(x, y, idx))
                idx += 1

        self.create_edges(self.distance_threshold)

    def get_closest_node(self, xy):
        """Find the node closest to a given (x, y) point."""
        # Validate if there are any nodes
        if not self.nodes_:
            return None
        # Calculate distances and return the index of the node with the smallest distance
        distances = [np.linalg.norm([node.x - xy[0], node.y - xy[1]]) for node in self.nodes_]
        print(distances.index(min(distances)))
        return distances.index(min(distances))

    def find_connected_groups(self):
        """
        Assigns each node in the graph to a connected component group. Each group represents
        a set of nodes that are interconnected, directly or indirectly.
        """
        graph_search = GraphSearch(self)

        # Initialize all nodes as not belonging to any group (0 --> no group)
        groups = [0] * len(self.nodes_)
        current_group = 1

        for idx in range(len(self.nodes_)):
            if groups[idx] == 0:
                connected_nodes = graph_search.find_connected_nodes(idx)
                for connected_node_idx in connected_nodes:
                    groups[connected_node_idx] = current_group
                current_group += 1
        self.groups_ = groups

    def create_point(self, node, z_offset=0.01):
        """
        Helper function to create a ROS Point from a node's pixel coordinates with an optional Z offset,
        converting pixel coordinates to world coordinates.
        """
        world_coordinates = self.map_.pixel_to_world(node.x, node.y)
        return Point(world_coordinates[0], world_coordinates[1], z_offset)

    def visualize_graph(self):
        """
        Visualizes the graph's nodes and edges using ROS markers. Nodes are colored based on their group,
        and edges are displayed between connected nodes.
        """
        rospy.sleep(0.5)
        self.marker_nodes_.points = []
        self.marker_nodes_.colors = []
        self.marker_edges_.points = []

        if self.groups_ is None:
            self.visualize_nodes_without_groups()
        else:
            self.visualize_nodes_with_groups()

        self.visualize_edges()  # Visualize all edges

        rospy.sleep(0.5)

    def visualize_nodes_without_groups(self):
        """Visualize nodes without any group distinction."""
        for node_i in self.nodes_:
            self.marker_nodes_.points.append(self.create_point(node_i))
        self.marker_pub_.publish(self.marker_nodes_)

    def visualize_nodes_with_groups(self):
        """Visualize nodes with group-based colors."""
        cmap = cm.get_cmap('Set1')
        colors = cmap.colors[0:-2]

        if self.show_conn:
            self.marker_nodes_.scale.x = .06
            self.marker_nodes_.scale.y = .06
            self.marker_nodes_.scale.z = .06

        for idx, node in enumerate(self.nodes_):
            color_idx = self.groups_[idx] % len(colors)
            color = ColorRGBA(*colors[color_idx], 1.0)
            self.marker_nodes_.points.append(self.create_point(node))
            self.marker_nodes_.colors.append(color)
        self.marker_pub_.publish(self.marker_nodes_)

    def visualize_edges(self):
        """Visualizes the edges between connected nodes."""
        for node_i in self.nodes_:
            for node_j in node_i.neighbours:
                self.marker_edges_.points.extend([self.create_point(node_i), self.create_point(node_j)])
        self.marker_pub_.publish(self.marker_edges_)

    def visualize_search(self, visited_set, unvisited_set, start_idx, goal_idx):
        """Visualizes the search process by marking visited and unvisited nodes, as well as the start and goal nodes."""
        # Clear previous points from the markers
        self.marker_visited_.points = []
        self.marker_unvisited_.points = []
        self.marker_start_.points = []

        # Update visited nodes
        for idx in visited_set:
            node = self.nodes_[idx]
            self.marker_visited_.points.append(self.create_point(node, z_offset=0.05))

        # Update unvisited nodes
        for idx in unvisited_set:
            node = self.nodes_[idx]
            self.marker_unvisited_.points.append(self.create_point(node, z_offset=0.05))

        # Update start and goal nodes
        start_node = self.nodes_[start_idx]
        goal_node = self.nodes_[goal_idx]
        self.marker_start_.points.append(self.create_point(start_node, z_offset=0.05))
        self.marker_start_.points.append(self.create_point(goal_node, z_offset=0.05))

        # Publish all markers
        self.marker_pub_.publish(self.marker_visited_)
        self.marker_pub_.publish(self.marker_unvisited_)
        self.marker_pub_.publish(self.marker_start_)

    def visualize_path(self, path):
        """Publishes a ROS path message visualizing the given path."""
        msg = self.prepare_path_message(path, z_offset=0.1)
        self.path_pub.publish(msg)

    def visualize_path_smooth(self, path):
        """Publishes a ROS path message visualizing the smoothed path."""
        msg = self.prepare_path_message(path, z_offset=0.12)
        self.path_smooth_pub.publish(msg)

    def prepare_path_message(self, path, z_offset=0.1):
        """Prepares a ROS path message from a list of nodes with a z offset."""
        msg = Path()
        msg.header.frame_id = 'map'
        for node in path:
            world_coordinates = self.map_.pixel_to_world(node.x, node.y)
            pose = PoseStamped()
            pose.pose.position.x = world_coordinates[0]
            pose.pose.position.y = world_coordinates[1]
            pose.pose.position.z = z_offset
            pose.pose.orientation.w = 1.0
            pose.header.frame_id = 'map'
            msg.poses.append(pose)
        return msg
