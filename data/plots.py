import json
import math
import re
import rosbag2_py 
import matplotlib.pyplot as plt
import pandas as pd
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


df = pd.json_normalize(json.load(open('data.json')), record_path="/dynamic_joint_states", meta=["header","stamp","sec"])
print(df.head(5))
