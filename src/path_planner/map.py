import rospy
import cv2 as cv
from geometry_msgs.msg import PoseStamped


class Map:
    def __init__(self):

        # Load map image from file specified in ROS parameter
        filename = rospy.get_param('~filename')
        self.image_ = cv.imread(filename, cv.COLOR_BGR2GRAY)
        if self.image_ is None:
            rospy.logerr(f"Failed to load map file at {filename}")
            return

        # Set map dimensions
        self.min_x_ = 0
        self.min_y_ = 0
        self.max_x_, self.max_y_ = self.image_.shape

        if len(self.image_.shape) == 3:
            self.image_ = self.image_[:,:,0]

        # Rviz subscriber
        self.rviz_goal_sub_ = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback, queue_size=1)
        self.rviz_goal = None

    def pixel_to_world(self, x, y, resolution=0.01):
        """ Convert pixel coordinates to world coordinates. """
        return [y*resolution, (self.max_x_-x)*resolution]

    def world_to_pixel(self, x, y, resolution=0.01):
        """ Convert world coordinates back to pixel coordinates. """
        return [self.max_x_ - y / resolution, int(x / resolution)]

    def rviz_goal_callback(self, msg):
        """ Handle new goal received from RViz. """
        self.rviz_goal = self.world_to_pixel(msg.pose.position.x, msg.pose.position.y)
        rospy.loginfo(f"New goal received: {self.rviz_goal}")

    def is_occupied(self, x, y):
        """ Check if the specified pixel coordinate is occupied. """
        if not (0 <= x < self.max_x_ and 0 <= y < self.max_y_):
            return True  # Out of bounds is considered occupied
        return self.image_[x, y] <= 235
