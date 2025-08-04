import numpy as np

def parse_matrix(matrix_str):
    """
    将输入的字符串格式的 4x4 矩阵解析为 numpy.ndarray。

    参数:
    matrix_str (str): 输入的矩阵字符串，支持标准小数和科学计数法格式。

    返回:
    np.ndarray: 4x4 的齐次变换矩阵。
    """
    # 去除首尾的方括号和多余空格
    cleaned = matrix_str.strip().replace('[', '').replace(']', '')

    # 统一换行符为逗号
    cleaned = cleaned.replace('\n', ',')

    # 按逗号分割，去除空白项
    elements = [x.strip() for x in cleaned.split(',') if x.strip()]

    # 检查是否正好有 16 个元素
    if len(elements) != 16:
        raise ValueError(f"输入矩阵应包含 16 个元素，但实际为 {len(elements)} 个。")

    # 转换为浮点数
    float_elements = [float(x) for x in elements]

    # 转换为 4x4 矩阵
    return np.array(float_elements).reshape(4, 4)

def multiply_transforms(transforms):
    """
    计算多个齐次变换矩阵的乘积。

    参数:
    transforms (list of np.ndarray): 4x4变换矩阵的列表。

    返回:
    np.ndarray: 组合后的4x4变换矩阵。
    """
    if not transforms:
        return np.eye(4)
    
    result = transforms[0].copy()
    for t in transforms[1:]:
        result = np.dot(t, result)  # 每个矩阵左乘当前结果
    return result

def main():
    

    # 示例矩阵 imu T cam 6（科学计数法格式）
    T_ab_str = """
    [0.,0.,1.,0.48,
         -1.,0.,0.,-0.06,
         0.,-1.,0.,-0.155,
         0.,0.,0.,1.]
    """
    # 示例矩阵 cam6 T cam 7（标准小数格式）
    # T_bc_str = """
    # [1.00000,0.00000,0.00600,-0.13100,
    #  0.00000,1.00000,-0.01400,0.00100,
    #  0.00600,0.01400,1.00000,0.00800,
    #  0.00000,0.00000,0.00000,1.00000]
    # """
    # cam6T cam7
    # """
    # 0.00600   -1.00000  0.00000   -0.13100  
    # -0.01400  0.00000   -1.00000  0.00100   
    # 1.00000   -0.00600  -0.01400  0.00800   
    # 0.00000   0.00000   0.00000   1.00000   
    # """
    T_bc_str = """
    [0.006, -1.00000, 0.00000, -0.13100,
     -0.01400, 0.00000, -1.00000, 0.00100,
     1.00000, -0.00600, -0.01400, 0.00800,
     0.00000, 0.00000, 0.00000, 1.00000]
    """

     # 示例矩阵 imu T cam 6（科学计数法格式）
    # T_ab_str = """
    # [1.00000,0.00000,0.0000,-0.13100,
    #  0.00000,1.00000,0.000,0.00200,
    #  0.0000,0.000,1.00000,0.00800,
    #  0.00000,0.00000,0.00000,1.00000]
    # """
    # # 示例矩阵 cam6 T cam 7（标准小数格式）
    # T_bc_str = """
    # [1.00000,0.00000,0.0000,0.13100,
    #  0.00000,1.00000,0.000,0.00100,
    #  0.0000,0.000,1.00000,0.00800,
    #  0.00000,0.00000,0.00000,1.00000]
    # """
    
    # 解析输入字符串为 numpy 矩阵
    T_ab = parse_matrix(T_ab_str)
    T_bc = parse_matrix(T_bc_str)

    # 计算 a -> c 的变换矩阵
    T_ac = multiply_transforms([T_ab, T_bc])

    print("a -> c 的变换矩阵 T_ac:")
    print(T_ac)

if __name__ == "__main__":
    main()