# 第一次尝试：gmapping

下载源码之后发现缺少tf变换数据和里程计发布的位姿数据，遂尝试使用rf2o发布里程数据（和odom->base_footprint），然后创建一个功能包来发布base_footprint->base_link->laser_link的tf固定变换

然后发现不知道为什么rf2o的定位表现非常奇怪，里程数据完全不对，没搞懂（

# 第二次尝试：hector

发现hector不需要里程数据，自己就可以，所以就用hector进行建图

结果保存在/result文件夹中
