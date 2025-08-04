#!/bin/bash

# 设置超时时间（秒）
TIMEOUT=2

# 临时文件
TMP_FILE=$(mktemp)
NO_DATA_FILE=$(mktemp)

# 获取所有非系统话题
ALL_TOPICS=$(ros2 topic list | grep -vE "^/(parameter_events|rosout)")

# 处理关键字筛选
if [ $# -gt 0 ]; then
  echo "筛选话题关键字: $1"
  TOPICS=$(echo "$ALL_TOPICS" | grep -i "$1")
else
  TOPICS="$ALL_TOPICS"
fi

# 检查话题数量
if [ -z "$TOPICS" ]; then
  echo "没有匹配的话题"
  exit 1
fi

# 开始检查
echo "开始检查以下话题（超时时间：${TIMEOUT}秒）："
echo "$TOPICS" | sed 's/^/  ➤ /'

for topic in $TOPICS; do
  # 使用timeout限制ros2命令执行时间
  if timeout $TIMEOUT ros2 topic echo "$topic" --once &> $TMP_FILE; then
    echo "✓ $topic 有数据反馈"
  else
    echo "✗ $topic 无数据反馈 ❗"
    echo "$topic" >> "$NO_DATA_FILE"
  fi
done

# 输出最终结果
echo -e "\n------ 检查完成 ------"
if [ -s "$NO_DATA_FILE" ]; then
  echo "以下话题没有数据反馈："
  cat "$NO_DATA_FILE" | sed 's/^/  - /'
else
  echo "所有话题均有数据反馈 ✅"
fi

# 清理临时文件
rm -f "$TMP_FILE" "$NO_DATA_FILE"