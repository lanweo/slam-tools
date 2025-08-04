# position_error

## 功能
分析 ROS2 bag 文件中的定位误差，生成误差分析图和数据表。

## 原理
读取 Odometry 和 GPS 数据，计算轨迹漂移，输出 CSV 和可视化图表。

## 使用示例
```python
python cal_position_error.py --bag your.bag --vins_topic /vins --gps_topic /gps
```
