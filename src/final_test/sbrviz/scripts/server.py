#!/usr/bin/env python3
import socket
import rospy
from geometry_msgs.msg import PoseStamped

# 连接后处理
def handle_client_connection(client_socket):
    # 创建pub
    pub = rospy.Publisher('/neu', PoseStamped, queue_size=10)
    # 初始化一个goal_forwarder节点
    rospy.init_node('goal_forwarder', anonymous=True)
    rate = rospy.Rate(10)

    # 监听数据
    while not rospy.is_shutdown():
        data = client_socket.recv(1024)
        data = data.decode('utf-8').strip()
        values = data.split(',')
        # 4为goal，3为pose
        if len(values) == 4:
            position_x, position_y, orientation_z, orientation_w = map(float, values)
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.pose.position.x = position_x
            goal.pose.position.y = position_y
            goal.pose.orientation.z = orientation_z
            goal.pose.orientation.w = orientation_w
            pub.publish(goal)
            rospy.loginfo(f"Publish goal: {goal}")
        elif len(values) == 3:
            position_x, position_y, orientation_z = map(float, values)
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = position_x
            pose.pose.position.y = position_y
            pose.pose.orientation.z = orientation_z
            pub.publish(pose)
            rospy.loginfo(f"Publish pose: {pose}")

    client_socket.close()

# 监听链接
def listen_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(('0.0.0.0', 12345))
    server.listen(5)
    print("runnning on localhost:12345")

    while True:
        client_sock, address = server.accept()
        print(f"connection from {address}")
        handle_client_connection(client_sock)

if __name__ == "__main__":
    listen_server()