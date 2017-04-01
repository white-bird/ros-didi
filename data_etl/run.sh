#!/bin/bash 
# etl the image and pcd from bags.

function runcmd()
{
	cmd="$1"
    pid=$(ps -ef | grep "$cmd" | grep -v grep | awk '{print $2}')
    if [ "$pid" != "" ]; then
        kill -9 $pid
    fi
    nohup $cmd >/dev/null 2>&1 &
}

function killcmd()
{
	cmd="$1"
    pid=$(ps -ef | grep "$cmd" | grep -v grep | awk '{print $2}')
    if [ "$pid" != "" ]; then
        kill -9 $pid
    fi   
}

function process()
{
    filename=$1
    arr=(${filename//./ }) 
    mkdir ${arr[0]//'/'/} &> /dev/null
	cd ${arr[0]//'/'/}
	cmd1="rosrun yingzz imgListener image:=/image_raw"
    cmd2="rosrun yingzz velListener pcl:=/velodyne_points"
    runcmd "$cmd1"
    runcmd "$cmd2"
    bag="../"${filename}
    echo $bag
    rosbag play $bag
    sleep 30
    killcmd imgListener
    killcmd velListener
    cd ..
}

runcmd "roscore"
sleep 5

#process target

process intersection_1.bag
process corner_pass.bag

#process all

#for filename in ./*.bag
#do
#    process $filename
#done
killcmd "roscore"
killcmd "rosout"
killcmd "rosmaster"
