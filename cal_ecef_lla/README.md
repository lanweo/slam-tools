# cal_ecef_lla

## 功能
将 ECEF（地心地固坐标系）坐标转换为 LLA（经纬度和高程）。

## 原理
基于 WGS-84 椭球参数，利用数学公式将三维空间坐标转换为地理坐标。

## 使用示例
```python
x = -2254505.4828
y = 5009248.4443
z = 3230331.3275
lat, lon, h = ecef_to_lla(x, y, z)
print(f"Latitude: {lat}°")
print(f"Longitude: {lon}°")
```
