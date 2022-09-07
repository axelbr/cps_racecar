from collections import defaultdict
import json
import math
import re
import dataclasses
import rosbag2_py 
import matplotlib.pyplot as plt
import pandas as pd
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_csv, message_to_ordereddict


def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)
    return storage_options, converter_options

storage, converter = get_rosbag_options('rosbags/hallway_limit_current_50/hallway_limit_current_50_0.db3')
reader = rosbag2_py.SequentialReader()
reader.open(storage, converter)

topic_types = reader.get_all_topics_and_types()

# Create a map for quicker lookup
type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

messages = defaultdict(list)
while reader.has_next():
    (topic, data, t) = reader.read_next()
    msg_type = get_message(type_map[topic])
    print(topic)
   
    msg = deserialize_message(data, msg_type)
    messages[topic].append(message_to_ordereddict(msg))
    print(message_to_csv(msg))


path = 'data'
print(messages)
with open(f'{path}.json', 'w+') as file:
    json.dump(messages, file)

