### terminal 1
    export ROS_MASTER_URI=http://localhost:11350
    export GAZEBO_MASTER_URI=http://localhost:11340
    roslaunch ubiquitous_display_gazebo train_world_2.launch

### terminal 2
    export ROS_MASTER_URI=http://localhost:11361
    export GAZEBO_MASTER_URI=http://localhost:11351
    roslaunch ubiquitous_display_gazebo train_world_2.launch

### Execute
    roscd distributed_dqn/scripts/
    python apex_dqn_agent.py --num_actors 2 --env_name UD-v1 --replay_memory_size 200000
    [testing mode]  python apex_dqn_agent.py --num_actors 1 --env_name UD-v1 --train 0 --load 1 --replay_memory_size 200000


## 実機実験のコマンド

#### 手順1
`~/.bashrc`の中の環境変数を変更する
- export ROS_MASTER_URI=http://10.42.0.1:11311/
- export ROS_IP=10.42.0.1

#### 手順2
メインPCで`roscore`を実行する
    roscore

#### 手順3
youbotのPCでURDFモデルやjspなどをlaunchする
    roslauunch youbot_driver_ros_interface youbot_driver_for_ud.launch

#### 手順4
LRFの起動
    sudo chmod o+wr /dev/tttyACM0
    sudo chmod o+wr /dev/tttyACM1
    rosrun urg_node getID /deev/ttyACM0
    rosrun urg_node getID /deev/ttyACM1
    roslaunch dual_ubiquitous_display_bringup dual_ubiquitous_display_lidar.launch

#### 手順5
人物検出ノードの起動
###### Termiinal 1
    source openvino_ws/devel/setup.bash
    roslaunch scan_image_ros open_camera_to_kmeans_node.launch

###### Termiinal 2
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/build/intel64/Release/lib
    source /opt/intel/computer_vision_sdk/bin/setupvars.sh
    roslaunch vino_launch pipeline_object_oss_topic.launch

###### Termiinal 3
    source catkin_ws/devel/setup.bash
    roslaunch leg_detector leg_detector.launch
###### Termiinal 4
    source catkin_ws/devel/setup.bash
    rosrun leg_detector expreiment_target_human.py
###### Termiinal 5 (Optional): Rviz marker node for taget human
    source catkin_ws/devel/setup.bash
    rosrun distributed_dqn mimic_target_human

###### Termiinal 6
    source catkin_ws/devel/setup.bash
    rosrun distributed_dqn apex_dqn_agent
