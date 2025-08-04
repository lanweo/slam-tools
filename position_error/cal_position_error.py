import rosbag2_py
from rclpy.serialization import deserialize_message
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from bisect import bisect
from matplotlib.collections import LineCollection
from matplotlib.cm import ScalarMappable
import os
from pathlib import Path

def get_message(type_name):
    from rosidl_runtime_py.utilities import get_message
    return get_message(type_name)

def read_bag_data(bag_path, vins_topic, gps_topic):
    """读取ROS2 bag数据并提取Odometry信息"""
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    vins_data = []
    gps_data = []

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        if topic == vins_topic:
            msg_type = get_message(type_map[vins_topic])
            msg = deserialize_message(data, msg_type)
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            vins_data.append((
                ts,
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ))
        elif topic == gps_topic:
            msg_type = get_message(type_map[gps_topic])
            msg = deserialize_message(data, msg_type)
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            gps_data.append((
                ts,
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ))

    vins_data.sort(key=lambda x: x[0])
    gps_data.sort(key=lambda x: x[0])
    
    return vins_data, gps_data


# import bisect

def align_and_calculate(vins_data, gps_data):
    """
    1. 根据VINS的初始位置，将所有位置信息减去初始位置x，y值
    2. 根据GPS的初始位置，将所有位置信息减去初始位置x，y值
    3. 对每个VINS数据，遍历GPS中时间最近的点数据来对齐时间戳，记录当前GPS遍历位置实现加速
    4. 返回漂移率和对齐后的GPS数据
    """
    
    # 1. 获取VINS初始位置作为共同参考点
    vins_init = (vins_data[0][1], vins_data[0][2])  # 提取VINS第一个点的x,y坐标
    for i in range(len(vins_data)):
        vins_data[i] = (
            vins_data[i][0],  # 时间戳
            vins_data[i][1] - vins_init[0],  # x坐标减去初始位置
            vins_data[i][2] - vins_init[1],  # y坐标减去初始位置
            vins_data[i][3]  # z坐标保持不变
        )
    adjusted_vins = [
        (ts, pos[0], pos[1])  # 坐标系原点校准
        for (ts, *pos) in vins_data  # 解包时间戳和坐标（格式：ts, x, y, ...）
    ]
    
    # 2. 根据GPS的初始位置校准坐标
    gps_init = (gps_data[0][1], gps_data[0][2])  # 提取GPS初始坐标
    for i in range(len(gps_data)):
        gps_data[i] = (
            gps_data[i][0],  # 时间戳
            gps_data[i][1] - gps_init[0],  # x坐标减去初始位置
            gps_data[i][2] - gps_init[1],  # y坐标减去初始位置
            gps_data[i][3]  # z坐标保持不变
        )
    adjusted_gps = [
        (ts, gps[0], gps[1])  # 坐标系原点校准
        for (ts, *gps) in gps_data  # 解包时间戳和坐标（格式：ts, x, y, ...）
    ]
    
    # 3. 时间戳对齐优化：利用有序性进行滑动窗口匹配
    gps_ts = [t[0] for t in adjusted_gps]  # 提取GPS时间序列（已排序）
    results = []  # 存储漂移结果（时间戳, 漂移距离, vins_x, vins_y）
    aligned_gps = []  # 存储对齐后的GPS坐标
    aligned_vins = []
    last_idx = 0  # 记录当前GPS搜索起始位置
    
    for vins in adjusted_vins:
        vts = vins[0]  # 当前VINS时间戳
        
        # 从上次结束位置开始查找（利用时间序列有序性）
        current_idx = last_idx
        # 线性查找第一个大于等于当前VINS时间戳的GPS索引
        while current_idx < len(gps_ts) and gps_ts[current_idx] < vts:
            current_idx += 1
        
        # 候选GPS点：前一个和当前（若存在）
        candidates = []
        if current_idx > 0:
            candidates.append(adjusted_gps[current_idx-1])
        if current_idx < len(adjusted_gps):
            candidates.append(adjusted_gps[current_idx])
        
        # 选择时间戳最接近的候选点
        closest = min(candidates, key=lambda x: abs(x[0]-vts)) if candidates else (0,0,0)
        
        # 更新下次搜索起始位置（关键优化点）
        last_idx = current_idx
        
        # 计算平面漂移（欧氏距离）
        dx = abs(vins[1] - closest[1])
        dy = abs(vins[2] - closest[2])
        distance = (dx**2 + dy**2)**0.5  # 等价于np.hypot但无需依赖
        
        # 记录结果
        results.append((vts, distance, vins[1], vins[2]))
        aligned_gps.append((closest[1], closest[2]))  # 仅保留坐标
        aligned_vins.append((vins[1], vins[2]))  # 仅保留坐标
    
    return results, aligned_gps,aligned_vins

def plot_trajectory_comparison(ax, gps_traj, vins_traj):
    """
    Enhanced trajectory comparison visualization with coordinate system auto-fit
    :param ax: matplotlib axis object
    :param gps_traj: Aligned GPS trajectory coordinates [(x1,y1), (x2,y2)...]
    :param vins_traj: VINS trajectory coordinates [(x1,y1), (x2,y2)...]
    """
    # 提取坐标
    gps_x, gps_y = zip(*gps_traj) if gps_traj else ([], [])
    vins_x, vins_y = zip(*vins_traj) if vins_traj else ([], [])
    
    # 绘制轨迹
    ax.plot(gps_x, gps_y, 'r--', linewidth=1.5, label='GPS Trajectory')
    ax.plot(vins_x, vins_y, 'b-', linewidth=1.0, label='VINS Trajectory')
    
    # 自动适配坐标范围
    all_x = list(gps_x) + list(vins_x)
    all_y = list(gps_y) + list(vins_y)
    ax.set_xlim(min(all_x)-1, max(all_x)+1)
    ax.set_ylim(min(all_y)-1, max(all_y)+1)
    
    # 添加图例和标签
    ax.legend(loc='upper left', fontsize=8)
    ax.set_xlabel('X Coordinate (m)')
    ax.set_ylabel('Y Coordinate (m)')
    ax.set_title('Trajectory Comparison (GPS vs VINS)')
    ax.grid(True, linestyle='--', alpha=0.7)

def save_and_visualize(results, gps_traj,vins_traj, output_dir):
    """增强可视化包含数值标注和文件保存"""
    # 创建输出目录
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    
    # 保存CSV
    df = pd.DataFrame(results, columns=['timestamp', 'drift_rate', 'vins_x', 'vins_y'])
    csv_path = os.path.join(output_dir, 'drift_analysis.csv')
    df.to_csv(csv_path, index=False)
    total_duration = df['timestamp'].max() - df['timestamp'].min()
    # 计算统计量
    stats = {
        'mean': df['drift_rate'].mean(),
        'std': df['drift_rate'].std(),
        'max': df['drift_rate'].max(),
        'min': df['drift_rate'].min(),
        '95%': df['drift_rate'].quantile(0.95)
    }
    
    # 可视化设置
    plt.figure(figsize=(20, 14))
    
    # 轨迹可视化
    ax1 = plt.subplot(2, 3, 1)
    # plot_trajectory_comparison(ax1, df, gps_traj,vins_traj, stats, total_duration)
    plot_trajectory_comparison(ax1, gps_traj, vins_traj)
    
    # 时间序列图
    ax2 = plt.subplot(2, 3, 2)
    ax2.plot(df['timestamp'], df['drift_rate'])
    ax2.set_title(f'Drift Rate Over Time\nMax: {stats["max"]:.2f}m, Min: {stats["min"]:.2f}m')
    ax2.set_xlabel('Timestamp')
    ax2.set_ylabel('Drift Rate (m)')
    ax2.grid(True)
    
    # 统计量柱状图
    ax3 = plt.subplot(2, 3, 3)
    stats_df = pd.DataFrame.from_dict(stats, orient='index', columns=['Value'])
    
    # 创建过滤后的数据
    filtered_stats = stats_df.drop('95%')
    
    filtered_stats.plot(kind='bar', ax=ax3, legend=False)
    ax3.set_title('Statistical Measures')
    ax3.set_xticklabels(filtered_stats.index, rotation=45)  # 使用正确的索引
    
    # 添加数值标签
    for i, (index, row) in enumerate(filtered_stats.iterrows()):
        ax3.text(i, row.Value+0.05, f'{row.Value:.2f}', ha='center')
    
    # 分布图
    ax4 = plt.subplot(2, 3, 4)
    n, bins, patches = ax4.hist(df['drift_rate'], bins=30, density=True, alpha=0.7)
    ax4.set_title(f'Drift Rate Distribution (μ={stats["mean"]:.2f}, σ={stats["std"]:.2f})')
    ax4.set_xlabel('Drift Rate (m)')
    ax4.set_ylabel('Density')
    ax4.grid(True)
    
    # 箱线图和小提琴图
    ax5 = plt.subplot(2, 3, 5)
    ax5.boxplot(df['drift_rate'], showmeans=True)
    ax5.set_title('Boxplot with Mean')
    ax5.set_xticks([])
    
    ax6 = plt.subplot(2, 3, 6)
    ax6.violinplot(df['drift_rate'], showmeans=True, showmedians=True)
    ax6.set_title('Violin Plot')
    ax6.set_xticks([])
    
    # 保存图片
    plt.tight_layout()
    img_path = os.path.join(output_dir, 'drift_analysis.png')
    plt.savefig(img_path, dpi=300, bbox_inches='tight')
    plt.show()


if __name__ == '__main__':
    # 配置参数
    bag_path = '/home/autoware/vins_ws/bags/result_bags/0414_c_mono_1'
    vins_topic = '/odometry'
    gps_topic = '/fixposition/odometry_enu'
    output_dir = '/home/autoware/slam-tools/position_error/results/0414_c_mono_1'  # 指定输出目录
    
    # 处理数据
    vins, gps = read_bag_data(bag_path, vins_topic, gps_topic)
    drift_results, gps_aligned,vins_aligned = align_and_calculate(vins, gps)
    save_and_visualize(drift_results, gps_aligned,vins_aligned, output_dir)


    
