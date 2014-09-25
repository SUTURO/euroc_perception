#!/bin/bash
echo %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
echo %     MPE DEBUG CAPTURE     %
echo %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
echo 
echo Please ensure that:
echo  - That one EuRoC Task is running in Gazebo with our planning interface
echo  - That the perception pipeline is running
echo  - The TCP camera points to the object, where the pose estimation fails
echo 
echo Next, we will start rosrun suturo_manipulation_tools perception_to_planningscene.
echo This calls the MPE and displays the matched object, if a match has been found.
echo In addition, we will start a rosbag record process.
echo 
echo 
echo If everything is set up, press 'y'. Otherwise, hit CTRL+C to exit
echo !IMPORTANT! As soon as you see MPE SUCCESS or MPE unsuccessful. Hit CTRL+C to abort the recording !!!
echo After you have done that, please look into the Output of the perception_task1.launch terminal window and look for something similar to this:
echo [INFO] [projection_segmenter]  pcl::ModelCoefficients: 4
echo [INFO] [projection_segmenter]    0.000590592
echo [INFO] [projection_segmenter]    0.489672
echo [INFO] [projection_segmenter]    0.871906
echo [INFO] [projection_segmenter]    -0.648024
echo
echo Note this 4 values and send them together with /tmp/mpe_debug_output.bag to pmania

while true; do
    read -p "Start process? [y/n] " yn
    case $yn in
        [Yy]* ) break;;
        [Nn]* ) exit;;
        * ) echo "Please answer yes or no.";;
    esac
done

echo Starting ...
trap 'kill $(jobs -p)' EXIT
rosbag record /tf /suturo/object_cluster_cloud/0 /suturo/object_cluster_cloud/1 /suturo/object_cluster_cloud/2 /suturo/object_cluster_cloud/3 /suturo/object_cluster_cloud/5  /suturo/object_cluster_cloud/6 /suturo/euroc_tcp_cloud -O /tmp/mpe_debug_output.bag &
rosrun suturo_manipulation_tools perception_to_planningscene &
wait
