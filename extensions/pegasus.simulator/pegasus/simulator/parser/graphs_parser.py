# Copyright (c) 2023, Marcelo Jacinto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# Graphs that can be used with the vehicles
from pegasus.simulator.parser import Parser
from pegasus.simulator.logic.graphs import ROS2Camera, ROS2Tf, ROS2Odometry, ROS2Lidar


class GraphParser(Parser):
    def __init__(self):

        # Dictionary of available graphs to instantiate
        self.graphs = {
            "ROS2 Camera": ROS2Camera,
            "ROS2 Tf": ROS2Tf,
            "ROS2 Odometry": ROS2Odometry,
            "ROS2 Lidar": ROS2Lidar
        }

    def parse(self, data_type: str, data_dict):

        # Get the class of the graph
        graph_cls = self.graphs[data_type]

        # Create an instance of that graph
        return graph_cls(data_dict)