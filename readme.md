# 介绍
    本项目旨在提供一些公共脚本方便数值计算，如不同旋转转换、数据计算、统计等

# 欧拉角转换为旋转矩阵
    convert_rpy_matrix.py

将xyz（平移）和rpy（横滚、俯仰、偏航）形式的相对位姿转换为4×4齐次变换矩阵的步骤如下：

### **1. 理解输入参数**
- **平移向量**：`xyz = [x, y, z]`，表示沿X、Y、Z轴的位移。
- **欧拉角（RPY）**：`rpy = [roll, pitch, yaw]`，分别绕X、Y、Z轴的旋转角度（通常为弧度）。

---

### **2. 将欧拉角转换为旋转矩阵**
假设旋转顺序为 **ZYX（内在旋转，即Yaw→Pitch→Roll）**，则旋转矩阵 `R` 由以下步骤计算：

#### **a. 绕Z轴旋转（偏航/Yaw）**

R_z(\text{yaw}) = 
\begin{bmatrix}
\cos(\text{yaw}) & -\sin(\text{yaw}) & 0 \\
\sin(\text{yaw}) & \cos(\text{yaw}) & 0 \\
0 & 0 & 1
\end{bmatrix}


#### **b. 绕Y轴旋转（俯仰/Pitch）**

R_y(\text{pitch}) = 
\begin{bmatrix}
\cos(\text{pitch}) & 0 & \sin(\text{pitch}) \\
0 & 1 & 0 \\
-\sin(\text{pitch}) & 0 & \cos(\text{pitch})
\end{bmatrix}


#### **c. 绕X轴旋转（横滚/Roll）**

R_x(\text{roll}) = 
\begin{bmatrix}
1 & 0 & 0 \\
0 & \cos(\text{roll}) & -\sin(\text{roll}) \\
0 & \sin(\text{roll}) & \cos(\text{roll})
\end{bmatrix}


#### **d. 组合旋转矩阵**
按 **ZYX顺序** 相乘（注意：矩阵乘法顺序为右乘）：

R = R_z(\text{yaw}) \cdot R_y(\text{pitch}) \cdot R_x(\text{roll})


---

### **3. 构建4×4齐次变换矩阵**
将旋转矩阵 `R` 和平移向量 `xyz` 组合成齐次变换矩阵：

T = 
\begin{bmatrix}
R_{11} & R_{12} & R_{13} & x \\
R_{21} & R_{22} & R_{23} & y \\
R_{31} & R_{32} & R_{33} & z \\
0 & 0 & 0 & 1
\end{bmatrix}


---

### **注意事项**
1. **角度单位**：确保输入的 `roll`, `pitch`, `yaw` 是弧度（若使用角度，需先转换为弧度）。
2. **旋转顺序**：默认为ZYX（内在旋转）。若使用其他顺序（如XYZ），需调整旋转矩阵相乘顺序。
3. **方向约定**：不同领域可能使用不同旋转轴方向（如机器人学与航空领域），需确认坐标系定义。

通过上述步骤，可将任意xyz和rpy参数转换为标准的4×4齐次变换矩阵。

# continous_rpy_trans.py
## 问题阐述
连续的欧拉角转换，如有A到B，B到C的欧拉角转换，求A到C的欧拉角变换
## 解决方法
将欧拉角转换为旋转矩阵，通过旋转矩阵连乘再转换回欧拉角
