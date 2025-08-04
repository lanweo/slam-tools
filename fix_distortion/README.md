# fix_distortion

## 功能
从 ROS2 bag 文件中提取图像并利用 CameraInfo 去畸变，保存结果图片。

## 原理
读取 ROS2 图像和相机参数，使用 OpenCV 去畸变算法处理图像。

## 使用示例
```python
process_bag_image("your.bag", "result/", "/image_topic", "/camera_info_topic")
```
