
import numpy as np
import math

def create_transform_matrix(rpy, translation, k=4):
    """
    计算齐次变换矩阵，并输出保留k位小数的结果
    
    参数:
    rpy : list[float]
        包含三个欧拉角的列表，顺序为 [roll, pitch, yaw]，单位为度
    translation : list[float]
        包含三个平移量的列表，顺序为 [x, y, z]
    k : int, optional
        输出矩阵中小数点后保留的位数，默认为4
        
    返回:
    transform_matrix : numpy.ndarray
        4x4 齐次变换矩阵，保留k位小数
    """
    # 将角度转换为弧度
    roll, pitch, yaw = np.radians(rpy)
    x, y, z = translation

    # 计算旋转矩阵
    # 绕X轴旋转（Roll）
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    # 绕Y轴旋转（Pitch）
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    # 绕Z轴旋转（Yaw）
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # 总旋转矩阵（顺序为 ZYX，即先绕X，再绕Y，最后绕Z）
    R = Rz @ Ry @ Rx

    # 创建齐次变换矩阵
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = R
    transform_matrix[:3, 3] = [x, y, z]

    # 对矩阵进行四舍五入处理，保留k位小数
    transform_matrix = np.round(transform_matrix, k)

    return transform_matrix

if __name__ == "__main__":
    rpy = [90,0,90]  # 单位：度
    translation = [0.41, -0.07, -0.14]
    k = 4  # 保留小数位数
    matrix = create_transform_matrix(rpy, translation, k)
    
    # 将矩阵展平为一行，并用逗号分隔每个元素
    flat_matrix = ",".join(map(str, matrix.flatten()))
    print("转换矩阵（一行）：", flat_matrix)

