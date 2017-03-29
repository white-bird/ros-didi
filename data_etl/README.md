1. roscreate-pkg yingzz
2. roscd yingzz 
3. cp -arf <git>/data_etl/* ./*
4. make
5. <new Terminate> roscore

6. mkdir approach_1
7. cd approach_1
8. rosrun yingzz listener image:=/image_raw

9. <new Terminate> rosbag play approach_1.bag

rosrun nodelet nodelet standalone velodyne_pointcloud/CloudNodelet 
roslaunch velodyne_pointcloud cloud_nodelet.launch
