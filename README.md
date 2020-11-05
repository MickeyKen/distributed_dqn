### terminal 1
    export ROS_MASTER_URI=http://localhost:11350
    export GAZEBO_MASTER_URI=http://localhost:11340
    roslaunch ubiquitous_display_gazebo train_world_2.launch

### terminal 2
    export ROS_MASTER_URI=http://localhost:11361
    export GAZEBO_MASTER_URI=http://localhost:11351
    roslaunch ubiquitous_display_gazebo train_world_2.launch

### Execute
    python apex_dqn.py --num_actors 2 --env_name UD-v0 --replay_memory_size 200000
