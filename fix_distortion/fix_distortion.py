import os
import random
import numpy as np
import cv2
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image, CameraInfo

def image_msg_to_cv2(img_msg):
    """将ROS2 Image消息转换为OpenCV图像"""
    if img_msg.encoding != 'mono8':
        raise ValueError("Unsupported image encoding: must be 'mono8'")
    return np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width)

def process_bag_image(bag_path, save_dir, image_topic, cam_info_topic):
    os.makedirs(save_dir, exist_ok=True)
    
    # 初始化ROS2 bag读取器
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader.open(storage_options, converter_options)
    
    cam_info = None
    image_buffer = []
    
    # 第一次遍历：收集所有图像消息和最新的CameraInfo
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == cam_info_topic:
            cam_info = deserialize_message(data, CameraInfo)
        elif topic == image_topic:
            image_buffer.append(deserialize_message(data, Image))
    
    # 验证数据有效性
    if cam_info is None:
        raise ValueError(f"CameraInfo not found on topic {cam_info_topic}")
    if not image_buffer:
        raise ValueError(f"No images found on topic {image_topic}")
    
    # 随机采样最多10张图像
    sample_size = min(10, len(image_buffer))
    selected_images = random.sample(image_buffer, sample_size)
    print(f"Processing {len(selected_images)} images...")
    
    # 准备相机参数
    camera_matrix = np.array(cam_info.k).reshape(3, 3)
    dist_coeffs = np.array(cam_info.d)
    
    # 处理并保存图像
    undistorted_images = []
    for idx, img_msg in enumerate(selected_images):
        # 图像转换与校正
        raw_image = image_msg_to_cv2(img_msg)
        undistorted = cv2.undistort(raw_image, camera_matrix, dist_coeffs)
        
        # 保存结果
        save_path = os.path.join(save_dir, f'undistorted_{idx:03d}.png')
        cv2.imwrite(save_path, undistorted)
        undistorted_images.append(undistorted)
    
    # 可视化结果（最多显示前9张）
    plt.figure(figsize=(15, 10))
    display_num = min(9, len(undistorted_images))
    for i in range(display_num):
        plt.subplot(3, 3, i+1)
        plt.imshow(undistorted_images[i], cmap='gray')
        plt.title(f'Undistorted Image {i}')
        plt.axis('off')
    plt.tight_layout()
    plt.show()

def main():
    # 配置参数
    # bag_path = "/home/autoware/ros_ws/0523_top_cam7/0523_top_cam7_0.db3"        # 替换为实际bag路径（不含.bag后缀）
    # bag_path = "/home/autoware/ros_ws/0526_cam7_top/0526_cam7_top_0.db3"        # 替换为实际bag路径（不含.bag后缀）
    bag_path = "/home/autoware/ros_ws/0526_cam0_top/0526_cam0_top_0.db3"        # 替换为实际bag路径（不含.bag后缀）
    save_dir = "/home/autoware/slam-tools/fix_distortion/result2"  # 替换为实际保存目录
    image_topic = "/camera7/image_raw_gray"
    cam_info_topic = "/camera7/camera_info"
    
    process_bag_image(bag_path, save_dir, image_topic, cam_info_topic)

if __name__ == "__main__":
    main()