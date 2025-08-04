# tf_relate_tools

## 功能
提供多种空间变换相关工具，包括：
- get_trans_matrix_4x4.py：根据欧拉角和位移生成 4x4 变换矩阵。
- convert_rpy_matrix.py：欧拉角与旋转矩阵互转，生成齐次变换矩阵。
- continous_rpy_trans.py：连续欧拉角变换，支持多组角度累积。
- continous_matrix_trans.py：连续变换矩阵运算，支持字符串解析和矩阵乘法。

## 原理
基于线性代数和空间变换理论，利用 numpy 实现矩阵运算和欧拉角转换。

## 使用示例
```python
# 生成变换矩阵
create_transform_matrix([roll, pitch, yaw], [x, y, z])
# 欧拉角转矩阵
rpy_to_matrix(roll, pitch, yaw)
# 多组变换累积
multiply_transforms([T1, T2, T3])
```
