import numpy as np

def rpy_to_matrix(roll, pitch, yaw):
    # 计算绕各轴的旋转矩阵
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    # 组合旋转矩阵（ZYX顺序）
    R = Rz @ Ry @ Rx
    return R

def pose_to_transform_matrix(x, y, z, roll, pitch, yaw):
    R = rpy_to_matrix(roll, pitch, yaw)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T

def matrix_to_rpy(R):
    # 从旋转矩阵提取ZYX欧拉角
    eps = 1e-6
    pitch = np.arcsin(-R[2, 0])
    if np.abs(np.cos(pitch)) > eps:
        yaw = np.arctan2(R[1, 0], R[0, 0])
        roll = np.arctan2(R[2, 1], R[2, 2])
    else:
        # 万向节死锁处理（假设roll=0）
        yaw = 0
        roll = np.arctan2(R[0, 1], R[1, 1])
    return roll, pitch, yaw
if __name__ == "__main__":
    
    # 示例：输入xyz=(1,2,3)，rpy=(0.1, 0.2, 0.3)（弧度）

    xyz = [0.905480, -0.046888, 0.927573]
    rpy = [-0.022425, 0.018654, 0.041715]
    T = pose_to_transform_matrix(*xyz, *rpy)
    print("cam3, 4x4变换矩阵：\n", T)


    xyz = [0.901731, 0.089778, 0.923683]
    rpy = [-0.016967, 0.018008, 0.055545]
    T = pose_to_transform_matrix(*xyz, *rpy)
    print("cam4, 4x4变换矩阵：\n", T)