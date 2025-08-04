# check_ros_net

## 功能
追踪 ROS2 网络中的话题、节点发布/订阅关系，辅助调试 ROS2 通信结构。

## 原理
通过 ROS2 API 获取节点和话题信息，递归遍历并过滤系统话题，输出网络结构。

## 使用示例
```bash
python check_ros_net.py --start_topic /your_topic
```
