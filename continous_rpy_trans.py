import numpy as np

def rpy_to_matrix(roll, pitch, yaw):
    # 生成ZYX顺序的旋转矩阵
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
    return Rz @ Ry @ Rx

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

def exec_multi_convert(rpy_list):
    """
    依次执行多个欧拉角转换
    :param rpy_list: 欧拉角列表, 每个元素为(roll, pitch, yaw)
    :return: 最终欧拉角
    """
    final_rpy = rpy_list[0]
    cur_R = rpy_to_matrix(final_rpy[0], final_rpy[1], final_rpy[2])
    for rpy_id in range(len(rpy_list)-1):
        roll, pitch, yaw = rpy_list[rpy_id+1]
        R = rpy_to_matrix(roll, pitch, yaw)
        cur_R = R @ cur_R
    return matrix_to_rpy(cur_R)




if __name__ == "__main__":
    # 示例：输入A→B和B→C的欧拉角（弧度）
    rpy_ab = (0.1, 0.2, 0.3)
    rpy_bc = (0.2, 0.3, 0.4)
    rpy_cd = (0.3, 0.4, 0.5)

    tets_ret = exec_multi_convert([rpy_ab, rpy_bc, rpy_cd])
    print(f"A→C的欧拉角（弧度）: roll={tets_ret[0]:.4f}, pitch={tets_ret[1]:.4f}, yaw={tets_ret[2]:.4f}")
