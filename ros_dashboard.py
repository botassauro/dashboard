#!/usr/bin/env python

import dash
from dash import dcc
from dash import html
import dash_daq as daq
import dash_bootstrap_components as dbc
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_srvs.srv import Trigger, TriggerRequest

# Create a class that inherits from Node for ROS 2 integration
class RosDashboardNode(Node):

    def __init__(self):
        super().__init__('ros_dashboard')
        self.get_logger().info('ROS Dashboard is starting...')

        # Initialize shared data structure to zeroes
        self.shared_dict = {
            "GAUGE_1": 0,
            "GAUGE_2": 0,
            "GAUGE_3": 0,
            "GAUGE_4": 0,
            "GAUGE_5": 0,
            "GAUGE_6": 0,
            "GAUGE_7": 0,
            "GAUGE_8": 0,
        }

        # Initialize Dash app
        external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']
        self.app = dash.Dash(
            external_stylesheets=[dbc.themes.BOOTSTRAP],
            title='ROS Dashboard'
        )

        # Initialize ROS subscribers
        self.setup_subscribers()

        # Initialize ROS service clients
        self.init_button_service_clients()

    def setup_subscribers(self):
        qos_profile = QoSProfile(depth=10)

        # A subscriber for each visualization
        # - The topic name is grabbed from the JSON file
        # - The ROS message type is grabbed from the JSON file as a string and interpreted to the respective Python class

        # Implement the subscribers here using rclpy.create_subscription

        # Example:
        # self.subscriber1 = self.create_subscription(
        #     YourMessageType,  # Replace with your message type
        #     visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_1"]["topic_name"],
        #     self.callback1,
        #     qos_profile
        # )

    def init_button_service_clients(self):
        # Initialize ROS service clients for button pushes (send a ROS service TriggeRequest upon button push)

        # Example:
        # self.button_1_trigger_service_client = self.create_client(
        #     Trigger,
        #     visualization_descriptions["buttons"]["button_1"]["trigger_service_to_invoke"]
        # )
        # while not self.button_1_trigger_service_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service not available, waiting...')
        # self.button_2_trigger_service_client = self.create_client(
        #     Trigger,
        #     visualization_descriptions["buttons"]["button_2"]["trigger_service_to_invoke"]
        # )
        # ...
        
        def callback1(self, msg):
            # Callback for Subscriber 1
            pass

        def callback2(self, msg):
            # Callback for Subscriber 2
            pass

        def callback3(self, msg):
            # Callback for Subscriber 3
            pass

        def callback4(self, msg):
            # Callback for Subscriber 4
            pass

        def callback5(self, msg):
            # Callback for Subscriber 5
            pass

        def callback6(self, msg):
            # Callback for Subscriber 6
            pass

        def callback7(self, msg):
            # Callback for Subscriber 7
            pass

        def callback8(self, msg):
            # Callback for Subscriber 8
            pass

        def run_dashboard(self):
            self.get_logger().info('Starting ROS Dashboard')

            with open('config.json') as json_file:
                visualization_descriptions = json.load(json_file)

            # Setup the layout of the dashboard
            self.setup_dash_app()

            # Spin the Dash server
            self.app.run_server(host="0.0.0.0", debug=False, port=8050, threaded=True)

        def setup_dash_app(self):
            # Modify this function to set up your Dash app layout as required
            pass

def main():
    rclpy.init(args=None)
    ros_dashboard_node = RosDashboardNode()
    ros_dashboard_node.run_dashboard()
    rclpy.spin(ros_dashboard_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
