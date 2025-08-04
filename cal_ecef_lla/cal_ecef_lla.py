import math

a = 6378137.0          # 长半轴
f = 1/298.257223563    # 扁率
b = a * (1 - f)        # 短半轴
e_sq = 2*f - f**2      # 第一偏心率平方
e_p_sq = e_sq/(1 - e_sq)  # 第二偏心率平方

def ecef_to_lla(x, y, z):
    # 经度
    lam = math.atan2(y, x)
    
    # 辅助量
    p = math.sqrt(x**2 + y**2)
    theta = math.atan2(z * a, p * b)
    
    # 纬度（闭合公式）
    phi = math.atan2(z + e_p_sq * b * math.sin(theta)**3,
                     p - e_sq * a * math.cos(theta)**3)
    
    # 高度
    N = a / math.sqrt(1 - e_sq * math.sin(phi)**2)
    h = (p / math.cos(phi)) - N
    
    # 转换为度
    return math.degrees(phi), math.degrees(lam), h

if __name__ == "__main__":
    """
    - X: -2254505.4828
    - Y: 5009248.4443
    - Z: 3230331.3275
    """
    x = -2254505.4828
    y = 5009248.4443
    z = 3230331.3275
    # 计算经纬度和高度
    lat, lon, h = ecef_to_lla(x, y, z)
    print(f"Latitude: {lat}°")
    print(f"Longitude: {lon}°")
    print(f"Height: {h} m")


