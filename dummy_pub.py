#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  # Import the appropriate message type
import json

'''

This code is used to test the ROS Dashboard. It publishes fake data to the topics specified in the JSON file config.json


'''


#helper function
# -converts a string containing a class name into a python class
def get_class( kls ):
    parts = kls.split('.')
    module = ".".join(parts[:-1])
    m = __import__( module )
    for comp in parts[1:]:
        m = getattr(m, comp)
    return m


#helper function
# -converta a ROS class string name (e.g. "sensor_msgs/BatteryState") into an instantiable python class (sensor_msgs.msg._BatteryState.BatteryState)
def json_string_to_ros_class(json_pretty_string):
    split = json_pretty_string.split("/")
    ros_class_name = split[0] + ".msg._"+ split[1] +"." + split[1]
    return get_class(ros_class_name)

#load the json file containing user-editable ROS topics/types
with open('config.json') as json_file:
    visualization_descriptions = json.load(json_file)
    # buttons_descriptions = visualization_descriptions["buttons"]
    widgets_descriptions = visualization_descriptions["visualizations"]

# publish fake ROS messages to the appropriate topics/type specified in the JSON file. Runs at 10Hz
import rclpy
from std_msgs.msg import Int32  # Assuming you are using Int32 messages, modify the import as needed

def publish_dummy_data():
    rclpy.init(args=None)
    node = rclpy.create_node('ros_dashboard_dummy_publisher')

    pub1 = node.create_publisher(
        Int32,
        widgets_descriptions["fast_rate_small_circular_gauge_1"]["topic_name"],
        10
    )
    pub2 = node.create_publisher(
        Int32,
        widgets_descriptions["fast_rate_small_circular_gauge_2"]["topic_name"],
        10
    )
    pub3 = node.create_publisher(
        Int32,
        widgets_descriptions["fast_rate_small_circular_gauge_3"]["topic_name"],
        10
    )
    pub4 = node.create_publisher(
        Int32,
        widgets_descriptions["fast_rate_large_circular_gauge_1"]["topic_name"],
        10
    )
    pub5 = node.create_publisher(
        Int32,
        widgets_descriptions["fast_rate_large_circular_gauge_2"]["topic_name"],
        10
    )
    pub6 = node.create_publisher(
        Int32,
        widgets_descriptions["medium_rate_thick_meter_1"]["topic_name"],
        10
    )
    pub7 = node.create_publisher(
        Int32,
        widgets_descriptions["medium_rate_thick_meter_2"]["topic_name"],
        10
    )
    pub8 = node.create_publisher(
        Int32,
        widgets_descriptions["medium_rate_thick_meter_3"]["topic_name"],
        10
    )

    rate = node.create_rate(10)  # 10 Hz

    msg1 = Int32()
    msg2 = Int32()
    msg3 = Int32()
    msg4 = Int32()
    msg5 = Int32()
    msg6 = Int32()
    msg7 = Int32()
    msg8 = Int32()

    msg1_value = widgets_descriptions["fast_rate_small_circular_gauge_1"]["mininum_value"]
    msg2_value = widgets_descriptions["fast_rate_small_circular_gauge_2"]["mininum_value"]
    msg3_value = widgets_descriptions["fast_rate_small_circular_gauge_3"]["mininum_value"]
    msg4_value = widgets_descriptions["fast_rate_large_circular_gauge_1"]["mininum_value"]
    msg5_value = widgets_descriptions["fast_rate_large_circular_gauge_2"]["mininum_value"]
    msg6_value = widgets_descriptions["medium_rate_thick_meter_1"]["mininum_value"]
    msg7_value = widgets_descriptions["medium_rate_thick_meter_2"]["mininum_value"]
    msg8_value = widgets_descriptions["medium_rate_thick_meter_3"]["mininum_value"]

    while rclpy.ok():
        msg1_value += 1
        msg2_value += 1
        msg3_value += 1
        msg4_value += 1
        msg5_value += 1
        msg6_value += 1
        msg7_value += 1
        msg8_value += 1

        msg1.data = msg1_value % widgets_descriptions["fast_rate_small_circular_gauge_1"]["maximum_value"]
        msg2.data = msg2_value % widgets_descriptions["fast_rate_small_circular_gauge_2"]["maximum_value"]
        msg3.data = msg3_value % widgets_descriptions["fast_rate_small_circular_gauge_3"]["maximum_value"]
        msg4.data = msg4_value % widgets_descriptions["fast_rate_large_circular_gauge_1"]["maximum_value"]
        msg5.data = msg5_value % widgets_descriptions["fast_rate_large_circular_gauge_2"]["maximum_value"]
        msg6.data = msg6_value % widgets_descriptions["medium_rate_thick_meter_1"]["maximum_value"]
        msg7.data = msg7_value % widgets_descriptions["medium_rate_thick_meter_2"]["maximum_value"]
        msg8.data = msg8_value % widgets_descriptions["medium_rate_thick_meter_3"]["maximum_value"]

        pub1.publish(msg1)
        pub2.publish(msg2)
        pub3.publish(msg3)
        pub4.publish(msg4)
        pub5.publish(msg5)
        pub6.publish(msg6)
        pub7.publish(msg7)
        pub8.publish(msg8)

        hello_str = "%i %i %i %i %i %i %i %i" % (
            msg1.data, msg2.data, msg3.data, msg4.data, msg5.data, msg6.data, msg7.data, msg8.data)
        node.get_logger().info(hello_str)

        rate.sleep()

    rclpy.shutdown()

if __name__ == '__main__':
    publish_dummy_data()


if __name__ == '__main__':
    try:
        rclpy.init(args=None)
        publish_dummy_data()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()