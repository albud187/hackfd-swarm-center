### `sample_to_show_topic_list.py` ###
import rclpy
from rclpy.node import Node
import time

def get_topic_list():
    #rclpy.init()
    node_dummy = Node("_ros2cli_dummy_to_show_topic_list")
    time.sleep(2)
    topic_list = node_dummy.get_topic_names_and_types()
    node_dummy.destroy_node()
    #rclpy.shutdown()
    result_list = [t[0] for t in topic_list]
    return result_list



def filter_topics(all_topics, ns_pfx, topic_name):
    """
    inputs: list[str] all_topics, str ns_pfx, str topic_name
    outputs: list[str] result_list
    descriptions: filters topics by namespace and type
    """
    
    result_list = []
    for topic in all_topics:
        # Check if the topic starts with the namespace prefix and ends with the topic name
        if topic.startswith(ns_pfx) and topic.endswith(topic_name):
            result_list.append(topic)
    return result_list


def get_ns(input_topic):
    result_ns = "/"+ input_topic.split("/")[1]
    return result_ns

# topic_list = get_topic_list()
# print(topic_list)
# print(" ")
# fr_pose = filter_topics(topic_list, "/r", "RPY_pose")
# print(fr_pose)
# print(get_ns(fr_pose[0]))