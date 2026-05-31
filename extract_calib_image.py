#!/usr/bin/env python3
"""
辅助脚本：从ROS2 bag中提取图像
使用方法: python3 extract_calib_image.py <bag_path> <image_topic> <output_path>
"""

import sys
import cv2
from cv_bridge import CvBridge
from rosidl_runtime_py.utilities import get_message
import rosbag2_py as rosbag2_py
from rclpy.serialization import deserialize_message


def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)
    return storage_options, converter_options


def extract_first_image(bag_path, image_topic, output_path):
    """从ROS2 bag中提取第一张图像"""
    
    print(f"Opening bag: {bag_path}")
    storage_options, converter_options = get_rosbag_options(bag_path)
    
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}
    
    if image_topic not in type_map:
        print(f"ERROR: Topic '{image_topic}' not found in bag!")
        print(f"Available topics: {list(type_map.keys())}")
        return False
    
    bridge = CvBridge()
    image_extracted = False
    
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        
        if topic == image_topic:
            try:
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                
                # Convert ROS Image to OpenCV format
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                
                # Save the image
                cv2.imwrite(output_path, cv_image)
                print(f"✓ Image extracted successfully: {output_path}")
                print(f"  Image size: {cv_image.shape}")
                image_extracted = True
                break
                
            except Exception as e:
                print(f"ERROR processing image: {e}")
                return False
    
    if not image_extracted:
        print(f"ERROR: No image found in topic '{image_topic}'")
        return False
    
    return True


if __name__ == '__main__':
    if len(sys.argv) != 4:
        print("Usage: python3 extract_calib_image.py <bag_path> <image_topic> <output_path>")
        print("\nExample:")
        print("  python3 extract_calib_image.py ./calib_bag /camera/camera/color/image_raw ./image.png")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    image_topic = sys.argv[2]
    output_path = sys.argv[3]
    
    success = extract_first_image(bag_path, image_topic, output_path)
    sys.exit(0 if success else 1)
