#!/usr/bin/env python3
import re
import socket
import subprocess
import csv
import sys
# 定义正则表达式来匹配目标信息
goal_pattern = re.compile(
    r"Setting goal: Frame:map, Position\(([-\d.]+), ([-\d.]+), ([-\d.]+)\), Orientation\(([-\d.]+), ([-\d.]+), ([-\d.]+), ([-\d.]+)\)"
)

# 定义正则表达式来匹配 /initialpose 信息
initialpose_pattern = re.compile(
    r"Setting pose: ([-\d.]+) ([-\d.]+) ([-\d.]+) \[frame=map\]"
)



# 读取master文件中的IP地址
def get_master_ip(file_path):
 with open(file_path, 'r') as file:
  for line in file:
   if line.startswith('export ROS_HOSTNAME='):
    return line.split('=')[1].strip()
 return None

# 创建 socket 连接
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('0.0.0.0', 12345))

# 启动 rviz 并捕获其输出
rviz_config_path = 'sbrviz.rviz'
with open('sbrviz.txt', 'w') as rviz_output_file, open('sbrviz.csv', 'a', newline='') as csvfile:
 # process = subprocess.Popen(['stdbuf', '-oL', 'rviz'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
 process = subprocess.Popen(['stdbuf', '-oL', 'rviz', '-d', rviz_config_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
 csv_writer = csv.writer(csvfile)

 try:
  while True:
   # 从 rviz 捕获输出
   output = process.stdout.readline()
   if output:
    # 将输出写入文件
    rviz_output_file.write(output)
    rviz_output_file.flush()
    print(output.strip())

    # 匹配 goal_pattern
    match_goal = goal_pattern.search(output)
    if match_goal:
     values = match_goal.groups()
     position_x = values[0]
     position_y = values[1]
     orientation_z = values[5]
     orientation_w = values[6]
     # 构建要发送的数据
     data = f"{position_x},{position_y},{orientation_z},{orientation_w}\n" 
     # 通过 socket 发送数据
     sock.sendall(data.encode('utf-8'))
     print(f"Sent goal data: {data.strip()}")

    # 匹配 initialpose_pattern
    match_initialpose = initialpose_pattern.search(output)
    if match_initialpose:
     values = match_initialpose.groups()
     position_x = values[0]
     position_y = values[1]
     orientation_z = values[2]
     # 构建要发送的数据
     data = f"{position_x},{position_y},{orientation_z}\n"
     # 通过 socket 发送数据
     sock.sendall(data.encode('utf-8'))
     print(f"Sent initialpose data: {data.strip()}")

 except KeyboardInterrupt:
  print("Process interrupted.")
 finally:
  process.terminate()
  sock.close()
