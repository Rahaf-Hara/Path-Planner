#!/usr/bin/env python3

import rospy
from path_planner.map import Map
from path_planner.graph import Graph
from path_planner.graph_search import GraphSearch
from path_planner.path_smoother import PathSmoother

if __name__ == '__main__':
    rospy.init_node('path_planner')

    map_instance = Map()
    rospy.sleep(3.0)  # Sleep to allow RViz to start up

    graph = Graph(map_instance)

    show_conn = rospy.get_param("~show_connectivity")

    if not show_conn:
        startx = rospy.get_param("~startx")
        starty = rospy.get_param("~starty")
        goalx = rospy.get_param("~goalx")
        goaly = rospy.get_param("~goaly")

        graph_search = GraphSearch(graph, [startx, starty], [goalx, goaly])
        PathSmoother(graph, graph_search.path_)

        print("Plan finished! Click a new goal in rviz 2D Nav Goal.")

        while not rospy.is_shutdown():
            if map_instance.rviz_goal:
                startx, starty = goalx, goaly
                goalx, goaly = map_instance.rviz_goal
                map_instance.rviz_goal = None

                graph_search = GraphSearch(graph, [startx, starty], [goalx, goaly])
                PathSmoother(graph, graph_search.path_)

                print("Plan finished! Click a new goal in rviz 2D Nav Goal.")
            rospy.sleep(0.01)
    rospy.spin()
